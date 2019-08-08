#include "Safe_Network_tcp.h"

#include <boost/asio.hpp>
#include <iostream>
#include <boost/bind.hpp>
#include <mutex>
#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include <functional>
#include <unordered_map>

using namespace std;
namespace asio = boost::asio;
using asio::ip::tcp;
using namespace boost::system;

//デバッグ時はネットワーク状態を表示
#ifdef __DEBUG_
#define SHOW_NETWORK_STATUS
#endif

Safe_Client::Safe_Client()
{
	connect_clear = true;
	on_connect_func = nullptr;
}

inline Safe_Client::Safe_Client(boost::asio::io_service & io_service) : Client::Client(io_service)
{
	connect_clear = true;
	on_connect_func = nullptr;
}

Safe_Client::~Safe_Client()
{
}

bool Safe_Client::connect(const std::string _address, const std::string _port, bool wait, Connect_Func connect_func)
{

	asio::io_service io_service_tmp;
	tcp::resolver resolver(io_service_tmp);
	tcp::resolver::query query(_address, _port);

	tcp::resolver::iterator resolve_query;
	boost::system::error_code ec;

	//address, portを記憶
	pair_address = _address;
	pair_port = _port;

	//接続中は再接続しない
	if (is_connecting)
		return false;

	//バッファを消去(connect_clear==trueなら)
	if (connect_clear)
		send_data_.clear();

	//ソケットを閉じる
	if (socket_.is_open())
		socket_.close();

	resolve_query = resolver.resolve(query, ec);
	if (ec) {
		cout << ec.message() << endl;
		io_service_tmp.stop();
		return false;
	}
	auto address_in = resolve_query->endpoint().address();
	auto port_in = resolve_query->endpoint().port();

	tcp::endpoint endpoint(address_in, port_in);

	//接続フラグtrue
	is_connecting = true;
	socket_.async_connect(endpoint,
		boost::bind(&Safe_Client::on_connect,
			this,
			&socket_,
			asio::placeholders::error,
			connect_func
		)
	);

	//waitがtrueならここで待つ
	if (wait) {
		io_service_.reset();
		io_service_.run();
		if (!socket_.is_open())
			connect(pair_address, pair_port, wait, connect_func);	//接続エラーなら再接続
	}

	/*asio::async_connect(
	socket_,
	resolve_query,
	boost::bind(&Client::on_connect, this, asio::placeholders::error));
	*/
	return true;
}

void Safe_Client::on_connect(const boost::asio::ip::tcp::socket * socket, const boost::system::error_code & error, Connect_Func connect_func) {

	//接続フラグを切る
	is_connecting = false;
	if (error) {
		std::cout << "connect failed : " << error.message() << std::endl;
		if (socket_.is_open())
			socket_.close();	//ソケットは閉じる
		connect(pair_address, pair_port, false, connect_func);	//接続エラーなら再接続
	}
	else {
		std::cout << "connected" << std::endl;
	}

	if (connect_func)
		connect_func(&socket_, error);
}

// メッセージ送信
void Safe_Client::sendf(Send_TCP_Func send_func) {
	sendf(send_data_.ptr(), send_data_.size(), send_func);
	send_data_.clear();
}

void Safe_Client::sendf(const void * buffer, size_t size, Send_TCP_Func send_func)
{
	if (!socket_.is_open())
		return;
	if (size < 0)
		return;

	shared_ptr<vector<unsigned char>> send_buf(new vector<unsigned char>(size));
	memcpy((unsigned char*)send_buf->data(), buffer, size);	//バッファをコピー
	asio::async_write(
		socket_,
		asio::buffer(buffer, size),
		boost::bind(&Safe_Client::on_send, this,
			asio::placeholders::error,
			send_buf,
			send_func)
	);
}

void Safe_Client::on_send(const boost::system::error_code& error, shared_ptr<vector<unsigned char>> buff, Send_TCP_Func send_func)
{
	Buffer send_buf;
	if (error) {
		std::cout << "send failed: " << error.message() << std::endl;
		//ソケットを閉じる
		if (socket_.is_open())
			socket_.close();
		connect(pair_address, pair_port, false, on_connect_func);	//接続エラーなら再接続
	}
	else {
#ifdef SHOW_NETWORK_STATUS
		std::cout << "send correct!" << std::endl;
#endif
	}
	if (0 < buff->size())
		send_buf.regist(&(*buff)[0], buff->size());

	if (send_func)
		send_func(send_buf, 0, error);
}

void Safe_Client::start_receive(size_t recv_size, Recv_TCP_Func recv_func)
{
	if (!socket_.is_open())
		return;

	shared_ptr<vector<unsigned char>> recv_buf(new vector<unsigned char>(recv_size));
	boost::asio::async_read(
		socket_,
		asio::buffer(*recv_buf),
		asio::transfer_all(),
		boost::bind(&Safe_Client::on_receive,
			this,
			asio::placeholders::error,
			recv_buf,
			recv_func)
	);
}

int Safe_Client::available(boost::system::error_code* ec)
{
	if (!socket_.is_open())
		return -1;

	boost::system::error_code errc;

	int avail = socket_.available(errc);

	if (ec)
		* ec = errc;

	if (errc)
		avail = -1;

	return avail;
}

void Safe_Client::on_receive(const boost::system::error_code & error, std::shared_ptr<std::vector<unsigned char>> buff, Recv_TCP_Func recv_func)
{
	Buffer recv_buf;

	if (error && error != boost::asio::error::eof) {
		std::cout << "receive failed: " << error.message() << std::endl;
		//ソケットを閉じる
		if (socket_.is_open())
			socket_.close();
		connect(pair_address, pair_port, false, on_connect_func);	//接続エラーなら再接続
	}
	else {
		if (0 < buff->size())
			recv_buf.regist(&(*buff)[0], buff->size());
	}

	if (recv_func)
		recv_func(recv_buf, 0, error);
}

bool Safe_Client::read_nonblock(void * buffer, size_t size, boost::system::error_code * ec)
{
	//引数エラーは飛ばす
	if (!socket_.is_open())
		return boost::asio::error::not_connected;
	if (!buffer)
		return boost::asio::error::invalid_argument;
	if (size <= 0)
		return boost::asio::error::invalid_argument;

	boost::system::error_code errc;

	bool ret = Client::read_nonblock(buffer, size, &errc);

	if (ec)
		*ec = errc;

	if (errc) {	//受信失敗
		buffer = nullptr;	//受信失敗の場合はヌルポインタにする		//ソケットを閉じる
							//ソケットを閉じる
		if (socket_.is_open())
			socket_.close();
		connect(pair_address, pair_port, false, on_connect_func);	//接続エラーなら再接続
		return ret;	//受信失敗の場合はfalseを返す
	}

	return ret;
}

boost::system::error_code Safe_Client::read(void * buffer, size_t size)
{
	//引数エラーは飛ばす
	if (!socket_.is_open())
		return boost::asio::error::not_connected;
	if (!buffer)
		return boost::asio::error::invalid_argument;
	if (size <= 0)
		return boost::asio::error::invalid_argument;

	boost::system::error_code ec;

	ec = Client::read(buffer, size);

	if (ec) {	//受信失敗
				//ソケットを閉じる
		if (socket_.is_open())
			socket_.close();
		connect(pair_address, pair_port, false, on_connect_func);	//接続エラーなら再接続
	}

	return ec;
}

Safe_Server::Safe_Server() : Server::Server() {
	on_accept_func = nullptr;
}

Safe_Server::Safe_Server(ushort port_no) : Server::Server(port_no) {
	on_accept_func = nullptr;
}

Safe_Server::Safe_Server(boost::asio::io_service & io_service, ushort port_no) : Server::Server(io_service, port_no) {
	on_accept_func = nullptr;
}

/************************************************
*
*        Safe_Server
*
************************************************/
int Safe_Server::start_accept(bool wait, Accept_Func accept_func, int index)
{
	uint socket_index;

	if (!acceptor_.is_open())
		return -1;

	if (index < 0 || socket_.size() <= index) {	//インデックス外の時
		shared_ptr<tcp::socket> tmp(new tcp::socket(io_service_));
		socket_.push_back(tmp);
		send_data_.push_back(Buffer());
		socket_index = socket_.size() - 1;
	}
	else if (socket_[index]->is_open()) {		//既に利用中の時
		shared_ptr<tcp::socket> tmp(new tcp::socket(io_service_));
		socket_.push_back(tmp);
		send_data_.push_back(Buffer());
		socket_index = socket_.size() - 1;
	}
	else {
		//accept中のリストを検索して同一インデックスなら拒否
		for (auto&& itr : accepting_list)
			if (itr == index)
				return -1;	//accept中は拒否

		socket_index = index;
	}
	//accepting_listに登録
	accepting_list.push_back(socket_index);

	//送信バッファをクリア
	send_data_[socket_index].clear();

	boost::system::error_code ec;
	acceptor_.listen(128, ec);
	if (ec) {
		cout << "Server Listen:" << ec.message() << endl;
		return -1;
	}

	acceptor_.async_accept(
		*socket_[socket_index],
		boost::bind(&Safe_Server::on_accept,
			this,
			asio::placeholders::error,
			socket_index,
			accept_func
		));


	if (wait) {
		io_service_.reset();
		io_service_.run();
		if (!socket_[socket_index]->is_open())
			start_accept(wait, accept_func, index);	//接続エラーなら再接続
	}

	return socket_index;
}

//acceptorを閉じる
void Safe_Server::close_accept() {
	return Server::close_accept();
}

void Safe_Server::on_send(const boost::system::error_code & error, shared_ptr<vector<unsigned char>> buff, uint index, Send_TCP_Func send_func)
{
	Buffer send_buf;
	if (error) {
		std::cout << "send failed: " << error.message() << std::endl;
		//ソケットを閉じる
		if (socket_[index]->is_open())
			socket_[index]->close();
		start_accept(false, on_accept_func, index);	//接続エラーなら再接続
	}
	else {
#ifdef SHOW_NETWORK_STATUS
		std::cout << "send correct!" << std::endl;
#endif
	}
	if (0 < buff->size())
		send_buf.regist(&(*buff)[0], buff->size());

	if (send_func)
		send_func(send_buf, 0, error);
}

void Safe_Server::sendf(uint index, Send_TCP_Func send_func)
{
	sendf(index, send_data_[index].ptr(), send_data_[index].size(), send_func);
	send_data_[index].clear();
}

void Safe_Server::sendf(uint index, const void * buff, size_t size, Send_TCP_Func send_func)
{
	if (socket_.size() <= index)
		return;
	if (!socket_[index]->is_open())
		return;
	if (size < 0)
		return;

	shared_ptr<vector<unsigned char>> send_buf(new vector<unsigned char>(size));
	memcpy((unsigned char*)send_buf->data(), buff, size);	//バッファをコピー
	asio::async_write(
		*socket_[index],
		asio::buffer(buff, size),
		boost::bind(&Safe_Server::on_send, this,
			asio::placeholders::error,
			send_buf,
			index,
			send_func
		));
}

void Safe_Server::start_receive(uint index, size_t size, Recv_TCP_Func recv_func)
{
	if (socket_.size() <= index)
		return;
	if (!socket_[index]->is_open())
		return;
	if (size <= 0)
		return;

	shared_ptr<vector<unsigned char>> recv_buf(new vector<unsigned char>(size));
	boost::asio::async_read(
		*socket_[index],
		asio::buffer(*recv_buf),
		asio::transfer_all(),
		boost::bind(&Safe_Server::on_receive,
			this,
			asio::placeholders::error,
			recv_buf,
			index,
			recv_func
		));
}

void Safe_Server::on_receive(const boost::system::error_code & error, std::shared_ptr<std::vector<unsigned char>> buff, uint index, Recv_TCP_Func func)
{
	if (socket_.size() <= index)
		return;

	Buffer recv_buf;

	if (error && error != boost::asio::error::eof) {
		std::cout << "receive failed: " << error.message() << std::endl;
		//ソケットを閉じる
		if (socket_[index]->is_open())
			socket_[index]->close();
		start_accept(false, on_accept_func, index);	//接続エラーなら再接続
	}
	else {
		if (0 < buff->size())
			recv_buf.regist(&(*buff)[0], buff->size());
	}

	if (func)
		func(recv_buf, index, error);
}

int Safe_Server::available(uint index, boost::system::error_code* ec)
{
	//引数エラーは飛ばす
	if (socket_.size() <= index)
		return -1;
	if (!socket_[index]->is_open())
		return -1;

	boost::system::error_code errc;

	int avail = socket_[index]->available(errc);

	if (ec)
		* ec = errc;

	if (errc)
		avail = -1;

	return avail;
}

bool Safe_Server::read_nonblock(uint index, void * buffer, size_t size, boost::system::error_code * ec)
{
	//引数エラーは飛ばす
	if (socket_.size() <= index)
		return false;
	if (!socket_[index]->is_open())
		return false;
	if (!buffer)
		return false;
	if (size <= 0)
		return false;

	boost::system::error_code errc;

	bool ret = Server::read_nonblock(index, buffer, size, &errc);

	if (ec)
		*ec = errc;

	if (ec) {	//受信失敗
				//ソケットを閉じる
		if (socket_[index]->is_open())
			socket_[index]->close();
		start_accept(false, on_accept_func, index);	//接続エラーなら再接続
	}

	return ret;
}

boost::system::error_code Safe_Server::read(uint index, void * buffer, size_t size)
{
	//引数エラーは飛ばす
	if (socket_.size() <= index)
		return boost::asio::error::invalid_argument;
	if (!socket_[index]->is_open())
		return boost::asio::error::not_connected;
	if (!buffer)
		return boost::asio::error::invalid_argument;
	if (size <= 0)
		return boost::asio::error::invalid_argument;

	boost::system::error_code ec;

	ec = Server::read(index, buffer, size);

	if (ec) {	//受信失敗
				//ソケットを閉じる
		if (socket_[index]->is_open())
			socket_[index]->close();
		start_accept(false, on_accept_func, index);	//接続エラーなら再接続
	}

	return ec;
}

void Safe_Server::on_accept(const boost::system::error_code & error, uint index, Accept_Func func)
{
	//accepting_listから消去
	accepting_list.remove(index);

	if (error) {
		std::cout << "accept failed: " << error.message() << std::endl;
		//ソケットを閉じる
		if (socket_[index]->is_open())
			socket_[index]->close();
		start_accept(false, func, index);	//接続エラーなら再接続
	}
	else {
		std::cout << "accept correct!" << std::endl;
	}

	if (func)
		func(&(*socket_[index]), index, error);
}

void main_safe_net_tcp_test()
{
	char c;
	cout << "クライアント？サーバー？c/s\n>";
	cin >> c;
	if (c == 'c')
	{
		string server_name;
		string port_no;
		cout << "サーバ名\n>";
		cin >> server_name;
		cout << "ポート番号\n>";
		cin >> port_no;

		Safe_Client s_client;
		s_client.connect(server_name, port_no);

		thread t1([&] {s_client.run(); });


		//s_client.start_receive();

		char ch[15] = "";
		std::this_thread::sleep_for(std::chrono::seconds(1));
		s_client.send("ping");
		auto ec = s_client.read(ch, 6);
		if (!ec)
			cout << ch << endl;
		ec = s_client.read(ch, 6);
		if (!ec)
			cout << ch << endl;
		ec = s_client.read(ch, 6);
		if (!ec)
			cout << ch << endl;
		s_client.send(ch, 0);
		if (!ec)
			cout << ch << endl;

		if (ec)
			cout << "Error Failed !\n";

		std::this_thread::sleep_for(std::chrono::seconds(20));
		s_client.stop();
		t1.join();
	}
	else if (c == 's')
	{
		char ch[15];

		Safe_Server server(51000);

		server.start_accept();

		thread t1([&] {server.run(); });

		cout << "start" << endl;

		cout << ":::  1 -\n";
		server.read(0, ch, 3);
		cout << ":::  2 -\n";
		server.read(0, &ch[3], 2);
		cout << ch << endl;
		cout << ":::  3 -\n";

		std::this_thread::sleep_for(std::chrono::seconds(3));

		cout << ":::  4 -\n";
		server.sendf(0, "ping1", nullptr);
		server.sendf(0, "ping2", nullptr);
		server.sendf(0, "ping3", nullptr);

		std::this_thread::sleep_for(std::chrono::seconds(20));
		server.stop();
		t1.join();
	}
}