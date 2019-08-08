/*
* network.cpp
*
*  Created on: 2017/11/13
*      Author: c611621020
*/

#include "network_tcp.h"

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

//io_serviceの詳細はここを参照<http://yutopp.hateblo.jp/entry/2011/12/15/001518>
//上のショート版(マルチスレッドはこっちが近い)<https://qiita.com/sileader/items/74c667e51d4bb36cafab>
//以下のクラスの大もとはこっち<https://boostjp.github.io/tips/network/tcp.html>
/*
//バッファ
//コピーコンストラクタ
Buffer::Buffer(const Buffer& src){
mtx.lock();
data_ = src.data_;
mtx.unlock();
}

Buffer Buffer::operator=(const Buffer& src){
mtx.lock();
data_ = src.data_;
mtx.unlock();

return *this;
}

size_t Buffer::consume(size_t size, void* buffer, bool start_first){
mtx.lock();
if(data_.size() < size){
size = data_.size();
}
if(buffer){
if(start_first)
memcpy(buffer, &data_[0], size);
else
memcpy(buffer, &data_[data_.size() - size - 1], size);
}
if(start_first)
data_.erase(data_.begin(), data_.begin() + size);
else
data_.erase(data_.end() - size - 1, data_.end() - 1);

mtx.unlock();
return size;
}

size_t Buffer::clear(){
size_t size = data_.size();
data_.clear();
mtx.unlock();
return size;
}

//構造体などはこれで一発
template <class T>
size_t Buffer::regist(const T& data){
return regist((void*)&data, sizeof(T));
}

//カスタム
size_t Buffer::regist(void* src, size_t size){
mtx.lock();
unsigned int old_size = data_.size();
data_.resize(old_size + size);
memcpy(&data_[old_size], src, size);
mtx.unlock();
return data_.size();
}

//サイズ
size_t Buffer::size(){
mtx.lock();
size_t tmp = data_.size();
mtx.unlock();
return tmp;
}

//データポインタ
void* Buffer::ptr(){
mtx.lock();
void* tmp = &data_[0];
mtx.unlock();
return tmp;
}

//データ
std::vector<char> Buffer::data(){
mtx.lock();
auto tmp = data_;
mtx.unlock();
return tmp;
}
*/
/*
* Client
*
*/


Client::Client()
	: io_service_ptr_(new asio::io_service()),
	io_service_(*io_service_ptr_),
	socket_(io_service_)
{
	is_connecting = false;
}

Client::Client(asio::io_service& io_service)
	: io_service_(io_service),
	socket_(io_service)
{
	is_connecting = false;
}

bool Client::connect(std::string address, string port, bool wait, Connect_Func connect_func)
{
	asio::io_service io_service_tmp;
	tcp::resolver resolver(io_service_tmp);
	tcp::resolver::query query(address, port);

	tcp::resolver::iterator resolve_query;
	boost::system::error_code ec;

	//接続中は再接続しない
	if (is_connecting)
		return false;

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
		boost::bind(&Client::on_connect,
			this,
			asio::placeholders::error,
			connect_func
		)
	);

	//waitがtrueならここで待つ
	if (wait) {
		io_service_.reset();
		io_service_.run();
		if (!socket_.is_open())
			return false;
	}

	/*asio::async_connect(
	socket_,
	resolve_query,
	boost::bind(&Client::on_connect, this, asio::placeholders::error));
	*/
	return true;
}

void Client::on_connect(const boost::system::error_code& error, Connect_Func connect_func)
{
	//接続フラグを切る
	is_connecting = false;
	if (error) {
		std::cout << "connect failed : " << error.message() << std::endl;
		socket_.close();	//ソケットは閉じる
	}
	else {
		std::cout << "connected" << std::endl;
	}

	if (connect_func)
		connect_func(&socket_, error);
}
//送信バッファに追加(カスタム)
void Client::push_sendbuf(const void* data, size_t size) {
	send_data_.regist((void*)data, size);
}

// メッセージ送信
void Client::send()
{
	send(&send_data_.data()[0], send_data_.size());
	send_data_.clear();
}

// メッセージ送信
void Client::send(const void* buffer, size_t size)
{
	if (!socket_.is_open())
		return;
	if (size <= 0)
		return;

	asio::async_write(
		socket_,
		asio::buffer(buffer, size),
		boost::bind(&Client::on_send, this,
			asio::placeholders::error)
	);
}

// 送信完了
// error : エラー情報
// bytes_transferred : 送信したバイト数
void Client::on_send(const boost::system::error_code& error)
{
	if (error) {
		std::cout << "send failed: " << error.message() << std::endl;
	}
	else {
#ifdef SHOW_NETWORK_STATUS
		std::cout << "send correct!" << std::endl;
#endif
	}
}

// メッセージ受信
void Client::start_receive(size_t recv_size, Recv_TCP_Func recv_func)
{
	if (!socket_.is_open())
		return;

	shared_ptr<vector<unsigned char>> recv_buf(new vector<unsigned char>(recv_size));
	boost::asio::async_read(
		socket_,
		asio::buffer(*recv_buf),
		asio::transfer_all(),
		boost::bind(&Client::on_receive, this,
			asio::placeholders::error, recv_buf, recv_func));
}

// 送受信開始
void Client::run()
{
	shared_ptr<asio::io_service::work> w(new asio::io_service::work(io_service_));
	worker = w;

	io_service_.reset();
	io_service_.run();
}

// 送受信停止
boost::system::error_code Client::stop()
{
	boost::system::error_code ec;
	socket_.cancel(ec);
	io_service_.stop();
	worker.reset();

	return ec;
}


// 受信完了
// error : エラー情報
// bytes_transferred : 受信したバイト数
void Client::on_receive(const boost::system::error_code& error, shared_ptr<vector<unsigned char>> buff, Recv_TCP_Func recv_func)
{
	Buffer recv_buf;

	if (error && error != boost::asio::error::eof) {
		std::cout << "receive failed: " << error.message() << std::endl;
	}
	else {
		if (0 < buff->size())
			recv_buf.regist(&(*buff)[0], buff->size());
	}

	if (recv_func)
		recv_func(recv_buf, 0, error);
}


/* 受信データを受け取る(受け取るまで待つ)
* buffer: 受信データを格納するバッファ
* size  : 受け取るサイズ
* 戻り値 : 受信していたらtrue
*/
bool Client::read_nonblock(void * buffer, size_t size, boost::system::error_code * ec)
{
	boost::system::error_code errc;

	if (!buffer)
		return false;

	if (!socket_.is_open() || is_connecting)
		return false;

	if (socket_.available(errc) == 0) {
		if (ec)
			*ec = errc;
		return false;
	}

	//受信
	asio::read(socket_,
		asio::buffer(buffer, size),
		asio::transfer_all(),
		errc);

	if (ec)
		*ec = errc;

	if (errc) {	//受信失敗
		buffer = nullptr;	//受信失敗の場合はヌルポインタにする
		return false;	//受信失敗の場合はfalseを返す
	}

	return true;
}

/* 受信データを受け取る(受け取るまで待つ)
* index : 対象のクライアント
* buffer: 受信データを格納するバッファ
* size  : 受け取るサイズ
* 戻り値 : エラーコード
*/
boost::system::error_code Client::read(void* buffer, size_t size) {
	if (!buffer)
		return boost::asio::error::invalid_argument;

	if (!socket_.is_open())
		return boost::asio::error::not_connected;

	boost::system::error_code ec;

	asio::read(socket_,
		asio::buffer(buffer, size),
		asio::transfer_all(),
		ec);

	if (ec)	//受信失敗の場合はヌルポインタにする
		buffer = nullptr;

	return ec;
}

bool Client::is_open()
{
	//接続中でないことも条件
	return socket_.is_open() && !is_connecting;
}

/*
*
* Server
*
*/

Server::Server()
	: io_service_ptr_(new asio::io_service()),
	io_service_(*io_service_ptr_),
	acceptor_(io_service_)
{}

Server::Server(ushort port_no)
	: io_service_ptr_(new asio::io_service()),
	io_service_(*io_service_ptr_),
	acceptor_(io_service_, tcp::endpoint(tcp::v4(), port_no))
{}

Server::Server(asio::io_service& io_service, ushort port_no)
	: io_service_(io_service),
	acceptor_(io_service, tcp::endpoint(tcp::v4(), port_no))
{}

void Server::init(ushort port_no)
{
	tcp::endpoint endp(tcp::v4(), port_no);
	boost::system::error_code ec;

	acceptor_.open(tcp::v4(), ec);
	if (ec) {
		cout << "Server Open :" << ec.message() << endl;
		ec.clear();
	}

	acceptor_.bind(endp, ec);
	while (ec) {
		cout << "Server Bind :" << ec.message() << endl;
		if (ec != boost::asio::error::already_open || ec != boost::asio::error::address_in_use)
			break;	//既にOpenの場合以外は単純に何かのエラー
					//自分で自分に接続してゾンビを退散
		ec.clear();
		Client* tmp_client = new Client();
		tmp_client->connect(endp.address().to_string(), to_string(port_no));
		delete tmp_client;	//解放
		acceptor_.bind(endp, ec);
	}
}

int Server::start_accept(bool wait, Accept_Func accept_func)
{
	shared_ptr<tcp::socket> tmp(new tcp::socket(io_service_));
	socket_.push_back(tmp);

	boost::system::error_code ec;
	acceptor_.listen(128, ec);
	if (ec) {
		cout << "Server Listen:" << ec.message() << endl;
		return -1;
	}

	acceptor_.async_accept(
		*socket_[socket_.size() - 1],
		boost::bind(&Server::on_accept,
			this,
			asio::placeholders::error,
			socket_.size() - 1,
			accept_func
		));

	send_data_.push_back(Buffer());

	if (wait) {
		io_service_.reset();
		io_service_.run();
	}

	return socket_.size() - 1;
}

// 送受信開始
void Server::run()
{
	shared_ptr<asio::io_service::work> w(new asio::io_service::work(io_service_));
	worker = w;

	io_service_.reset();
	io_service_.run();
}

// 送受信停止
void Server::stop()
{
	//例外発生抑制
	boost::system::error_code ec;
	for (auto&& itr : socket_)
		itr->cancel(ec);
	io_service_.stop();
	worker.reset();
}

// メッセージ送信
/*    void sendf(string str)
{
cout << "sendf" << endl;
string send_data_ = str;
asio::async_write(
socket_[0],
asio::buffer(send_data_),
boost::bind(&Server::on_send, this,
asio::placeholders::error,
asio::placeholders::bytes_transferred));
}
*/
// 送信完了
// error : エラー情報
// bytes_transferred : 送信したバイト数
void Server::on_send(const boost::system::error_code& error)
{
	if (error) {
		std::cout << "send failed: " << error.message() << std::endl;
	}
	else {
#ifdef SHOW_NETWORK_STATUS
		std::cout << "send correct!" << std::endl;
#endif
	}
}

void Server::close_accept() {
	boost::system::error_code ec;
	acceptor_.close(ec);

	if (ec) {
		cout << "Error : " << ec.message() << endl;
	}
}

// メッセージ送信(単体)

//送信バッファに追加(カスタム)
void Server::push_sendbuf(uint index, const void* data, size_t size) {
	if (socket_.size() <= index)
		return;

	send_data_[index].regist((void*)data, size);
}

void Server::send(uint index) {
	send(index, send_data_[index].ptr(), send_data_[index].size());
	send_data_[index].clear();
}

// メッセージ送信
void Server::send(uint index, const void* buffer, size_t size)
{
	if (socket_.size() <= index)
		return;
	if (!socket_[index]->is_open())
		return;

	asio::async_write(
		*socket_[index],
		asio::buffer(buffer, size),
		boost::bind(&Server::on_send, this,
			asio::placeholders::error
		));
}

// メッセージ受信(最小サイズ指定)
void Server::start_receive(uint index, size_t size, Recv_TCP_Func recv_func)
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
		boost::bind(&Server::on_receive, this,
			asio::placeholders::error,
			recv_buf,
			index,
			recv_func
		));
}

// 受信完了
// error : エラー情報
// bytes_transferred : 受信したバイト数
void Server::on_receive(const boost::system::error_code& error, shared_ptr<vector<unsigned char>> buff, uint index, Recv_TCP_Func func)
{
	if (socket_.size() <= index)
		return;

	Buffer recv_buf;

	if (error && error != boost::asio::error::eof) {
		std::cout << "receive failed: " << error.message() << std::endl;
	}
	else {
		if (0 < buff->size())
			recv_buf.regist(&(*buff)[0], buff->size());
	}

	if (func)
		func(recv_buf, index, error);
}

bool Server::read_nonblock(uint index, void * buffer, size_t size, boost::system::error_code * ec)
{
	boost::system::error_code errc;
	if (socket_.size() <= index)
		return false;
	if (!buffer)
		return false;
	if (!socket_[index]->is_open())
		return false;

	if (socket_[index]->available(errc) < size) {
		if (ec)
			*ec = errc;
		return false;
	}

	//受信
	asio::read(*socket_[index],
		asio::buffer(buffer, size),
		asio::transfer_all(),
		errc);

	if (ec)
		*ec = errc;

	if (errc) {	//受信失敗
		buffer = nullptr;	//受信失敗の場合はヌルポインタにする
		return false;	//受信失敗の場合はfalseを返す
	}

	return true;
}

/* 受信データを受け取る(受け取るまで待つ)
* index : 対象のクライアント
* buffer: 受信データを格納するバッファ
* size  : 受け取るサイズ
* 戻り値 : 受信サイズ(エラーは-1)
*/
boost::system::error_code Server::read(uint index, void* buffer, size_t size) {
	if (socket_.size() <= index)
		return boost::asio::error::invalid_argument;
	if (!buffer)
		return boost::asio::error::invalid_argument;
	if (!socket_[index]->is_open())
		return boost::asio::error::not_connected;

	boost::system::error_code ec;
	asio::read(*socket_[index],
		asio::buffer(buffer, size),
		asio::transfer_all(),
		ec
	);

	if (!ec)	//受信失敗の場合はヌルポインタにする
		buffer = nullptr;

	return ec;
}

bool Server::is_open(uint index)
{
	if (socket_.size() <= index)
		return false;

	return socket_[index]->is_open();
}

void Server::on_accept(const boost::system::error_code& error, uint index, Accept_Func func)
{
	if (error) {
		std::cout << "accept failed: " << error.message() << std::endl;
	}
	else {
		std::cout << "accept correct!" << std::endl;
	}

	if (func)
		func(&(*socket_[index]), index, error);
}

void main_net_tcp_test()
{
	char c;
	cout << "クライアント？サーバー？c/s\n>";
	cin >> c;
	if (c == 'c')
	{
		char ch[15];
		string server_name;
		string port_no;
		cout << "サーバ名\n>";
		cin >> server_name;
		cout << "ポート番号\n>";
		cin >> port_no;

		Client client;
		client.connect(server_name, port_no);

		thread t1([&] {client.run(); });


		//client.start_receive();

		std::this_thread::sleep_for(std::chrono::seconds(1));
		client.send("ping");
		client.read(ch, 6);
		cout << ch << endl;
		client.read(ch, 6);
		cout << ch << endl;
		client.read(ch, 6);
		cout << ch << endl;

		client.stop();
		t1.join();
	}
	else if (c == 's')
	{
		char ch[15];

		Server server(51000);

		server.start_accept();

		thread t1([&] {server.run(); });

		cout << "start" << endl;

		cout << ":::  1 -\n";
		server.read(0, ch, 3);
		cout << ":::  2 -\n";
		server.read(0, &ch[3], 2);
		cout << ch << endl;

		std::this_thread::sleep_for(std::chrono::seconds(3));

		server.send(0, "ping1");
		server.send(0, "ping2");
		server.send(0, "ping3");

		server.stop();
		t1.join();
	}
}