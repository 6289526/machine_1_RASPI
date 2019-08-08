/*
* network.cpp
*
*  Created on: 2017/11/13
*      Author: c611621020
*/

#include "network.h"

#include "network_udp.h"

#include <boost/asio.hpp>
#include <boost/array.hpp>
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
using asio::ip::udp;

//デバッグ時はネットワーク状態を表示
#ifdef __DEBUG_
#define SHOW_NETWORK_STATUS
#endif

//io_serviceの詳細はここを参照<http://yutopp.hateblo.jp/entry/2011/12/15/001518>
//上のショート版(マルチスレッドはこっちが近い)<https://qiita.com/sileader/items/74c667e51d4bb36cafab>
//以下のクラスの大もとはこっち<https://boostjp.github.io/tips/network/udp.html>
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
* Net_udp
*
*/

Net_udp::Net_udp()
	: io_service_ptr_(new asio::io_service()),
	io_service_(*io_service_ptr_),
	socket_(io_service_)
{
	need_size = 0;
}

Net_udp::Net_udp(ushort port_no)
	: io_service_ptr_(new asio::io_service()),
	io_service_(*io_service_ptr_),
	socket_(io_service_, udp::endpoint(udp::v4(), port_no))
{
	need_size = 0;
}

Net_udp::Net_udp(asio::io_service& io_service, ushort port_no)
	: io_service_(io_service),
	socket_(io_service, udp::endpoint(udp::v4(), port_no))
{
	need_size = 0;
}


void Net_udp::init(ushort port_no)
{
	boost::system::error_code ec;
	socket_.open(udp::v4());
	socket_.bind(udp::endpoint(udp::v4(), port_no), ec);

	if (ec) {
		cout << "UDP INIT : " << ec.message() << endl;
	}
}
/*
void Net_udp::init(boost::asio::ip::udp::endpoint endp)
{
	boost::system::error_code ec;
	socket_.open(udp::v4());
	socket_.bind(udp::endpoint(udp::v4(), endp.port()), ec);

	if (ec) {
		cout << "UDP INIT : " << ec.message() << endl;
	}
	socket_.connect(endp, ec);
	if (ec) {
		cout << "UDP INIT : " << ec.message() << endl;
	}
}
*/
/*
bool Net_udp::connect(std::string address, string port,bool wait)
{
asio::io_service io_service_tmp;
udp::resolver resolver(io_service_tmp);
udp::resolver::query query(address, port);

udp::resolver::iterator resolve_query;
try{
resolve_query = resolver.resolve(query);
}catch(std::exception& e){
cout << e.what() << endl;
io_service_tmp.stop();
return false;
}
auto address_in = resolve_query->endpoint().address();
auto port_in = resolve_query->endpoint().port();

udp::endpoint endpoint(address_in, port_in);

socket_.async_connect(endpoint, boost::bind(&Net_udp::on_connect, this, asio::placeholders::error));

//waitがtrueならここで待つ
if(wait){
io_service_.reset();
io_service_.run();
if(!socket_.is_open())
return false;
}

recv_fin = false;
/*asio::async_connect(
socket_,
resolve_query,
boost::bind(&Net_udp::on_connect, this, asio::placeholders::error));
*//*
return true;
}
*/
/*
void Net_udp::on_connect(const boost::system::error_code& error)
{
if (error) {
std::cout << "connect failed : " << error.message() << std::endl;
socket_.close();	//ソケットは閉じる
}
else {
std::cout << "connected" << std::endl;
}
}
*/

//送信バッファに追加(カスタム)
void Net_udp::push_sendbuf(void* data, size_t size) {
	send_data_.regist(data, size);
}

// メッセージ送信
void Net_udp::send(boost::asio::ip::udp::endpoint send_endpoint)
{
	send(send_data_.ptr(), send_data_.size(), send_endpoint);
	send_data_.clear();
}

// メッセージ送信
void Net_udp::send(const void* data, size_t size, boost::asio::ip::udp::endpoint send_endpoint)
{
	//ブロードキャスト設定
	bool broadcast = false;
	if (send_endpoint.address() == asio::ip::address_v4::broadcast()) {
		broadcast = true;
		socket_.set_option(asio::socket_base::broadcast(true));
	}

	shared_ptr<Buffer> dummy;
	socket_.async_send_to(
		asio::buffer(data, size),
		send_endpoint,
		boost::bind(&Net_udp::on_send, this,
			asio::placeholders::error,
			dummy));

	if (broadcast)
		socket_.set_option(udp::socket::broadcast(false));
}

// 送信完了
// error : エラー情報
// bytes_transferred : 送信したバイト数
void Net_udp::on_send(const boost::system::error_code& error, shared_ptr<Buffer> buf)
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


// メッセージ送信(ブロッキング)
boost::system::error_code Net_udp::send_block(boost::asio::ip::udp::endpoint send_endpoint)
{
	boost::system::error_code ec;

	ec = send_block(send_data_.ptr(), send_data_.size(), send_endpoint);
	send_data_.clear();

	return ec;
}

// メッセージ送信(単体, ブロッキング)
boost::system::error_code Net_udp::send_block(const void * data, size_t size, boost::asio::ip::udp::endpoint send_endpoint)
{
	//ブロードキャスト設定
	bool broadcast = false;
	if (send_endpoint.address() == asio::ip::address_v4::broadcast()) {
		broadcast = true;
		socket_.set_option(asio::socket_base::broadcast(true));
	}

	boost::system::error_code ec;
	shared_ptr<Buffer> dummy;
	socket_.send_to(
		asio::buffer(data, size),
		send_endpoint,0,
		ec);

	if (broadcast)
		socket_.set_option(udp::socket::broadcast(false));

	return ec;
}

// メッセージ受信
void Net_udp::start_receive(Recv_UDP_Func recv_func)
{
	if (!socket_.is_open())
		return;

	shared_ptr<vector<unsigned char>> recv_buf(new vector<unsigned char>(UDP_DATA_SIZE));

	udp::endpoint recv_endp;


	socket_.async_receive_from(
		asio::buffer(*recv_buf),
		recv_endp,
		boost::bind(&Net_udp::on_receive, this,
			asio::placeholders::error, recv_buf, recv_endp, recv_func));

	/*
	boost::asio::async_read(
	socket_,
	*recv_buf,
	asio::transfer_at_least(1),
	boost::bind(&Net_udp::on_receive, this,
	asio::placeholders::error, recv_buf)
	);
	*/
}

// 送受信開始
void Net_udp::run()
{
	shared_ptr<asio::io_service::work> w(new asio::io_service::work(io_service_));
	worker = w;

	io_service_.reset();
	io_service_.run();
}

// 送受信停止
void Net_udp::stop(bool ReleaseMutex)
{
	socket_.cancel();
	io_service_.stop();
	worker.reset();

	//mutexも開放
	read_mutex_.notify_all();
}


// 受信完了
// error : エラー情報
// bytes_transferred : 受信したバイト数
void Net_udp::on_receive(const boost::system::error_code& error, shared_ptr<vector<unsigned char>> buff, udp::endpoint endp, Recv_UDP_Func func)
{
	if (error && error != boost::asio::error::eof) {
		std::cout << "receive failed: " << error.message() << std::endl;
	}
	else {
		Buffer buffer;
		buffer.regist((void*)&((*buff)[0]), buff->size());

		if (func)
			func(buffer, endp);
	}
}

/* 受信データを受け取る(バッファが溜まっていたら読み取る)
* buffer: 受信データを格納するバッファ
* clear : 受信バッファを全消去して上書き
* addr : 送信元アドレス
* 戻り値 : 受信データがあればtrue
*/
bool Net_udp::recv_nonblock(Buffer* buffer, bool clear, UDP_Address* addr) {
	if (!buffer)
		return false;

	if (!socket_.is_open())
		return false;

	//バッファをクリア
	if (clear)
		buffer->clear();
	//readを競合させないロック(sizeの書き換え防止)
	read_unique_mutex_.lock();
	{
		//現在のバッファをコピーして消費
		std::vector<unsigned char> dummy_buf(0);
		vector<unsigned char> all_recv_buf;
		udp::endpoint recv_endp;

		//受信サイズ取得
		size_t recv_bytes = socket_.available();
		if (recv_bytes == 0) {
			read_unique_mutex_.unlock();
			return false;
		}

		try {
			//受信サイズを取得,バッファを拡張
			all_recv_buf.resize(recv_bytes);
			//すべて読み込み
			socket_.receive_from(
				asio::buffer(all_recv_buf),
				recv_endp
			);
		}
		catch (std::exception& e) {
			cout << e.what() << endl;
		}
		if (addr)
			*addr = recv_endp;
		buffer->regist(&all_recv_buf[0], all_recv_buf.size());
	}
	read_unique_mutex_.unlock();

	return true;
}

bool Net_udp::recvsz_nonblock(Buffer * buffer, size_t sz, bool clear, UDP_Address * addr)
{
	if (!buffer)
		return false;

	if (!socket_.is_open())
		return false;

	//バッファをクリア
	if (clear)
		buffer->clear();
	//readを競合させないロック(sizeの書き換え防止)
	read_unique_mutex_.lock();
	{
		//現在のバッファをコピーして消費
		vector<unsigned char> all_recv_buf(sz);
		udp::endpoint recv_endp;

		//受信サイズ取得
		size_t recv_bytes = socket_.available();
		if (recv_bytes < sz) {
			read_unique_mutex_.unlock();
			return false;
		}

		try {
			//すべて読み込み
			socket_.receive_from(
				asio::buffer(all_recv_buf),
				recv_endp
			);
		}
		catch (std::exception& e) {
			cout << e.what() << endl;
		}
		if (addr)
			*addr = recv_endp;
		buffer->regist(&all_recv_buf[0], all_recv_buf.size());
	}
	read_unique_mutex_.unlock();

	return true;
}

bool Net_udp::recv_nonblock(bool clear, UDP_Address* addr) {
	return recv_nonblock(&recv_data_, clear, addr);
}

/* 受信データを受け取る(受け取るまで待つ)
* index : 対象のクライアント
* buffer: 受信データを格納するバッファ
* size  : 受け取るサイズ
* 戻り値 : 受信サイズ(エラーは-1)
*/
UDP_Address Net_udp::recv(Buffer* buffer, bool clear) {
	UDP_Address addr;
	if (!buffer)
		return addr;

	if (!socket_.is_open())
		return addr;

	//バッファをクリア
	if (clear)
		buffer->clear();
	//readを競合させないロック(sizeの書き換え防止)
	read_unique_mutex_.lock();
	{
		//現在のバッファをコピーして消費
		std::vector<unsigned char> dummy_buf(0);
		vector<unsigned char> all_recv_buf;
		udp::endpoint recv_endp;

		try {
			//一旦受信が来るまで待つ
			socket_.receive_from(
				asio::buffer(dummy_buf),
				recv_endp
				, asio::socket_base::message_peek
			);
			//受信サイズを取得,バッファを拡張
			all_recv_buf.resize(socket_.available());
			//すべて読み込み
			socket_.receive_from(
				asio::buffer(all_recv_buf),
				recv_endp
			);
		}
		catch (std::exception& e) {
			cout << e.what() << endl;
		}

		addr = recv_endp;
		buffer->regist(&all_recv_buf[0], all_recv_buf.size());
	}
	read_unique_mutex_.unlock();

	return addr;
}

UDP_Address Net_udp::recv(bool clear) {
	return recv(&recv_data_, clear);
}

bool Net_udp::read(Buffer& buffer, bool consume) {
	if (recv_data_.size() <= 0)
		return false;

	if (consume) {
		buffer.regist(recv_data_.ptr(), recv_data_.size());
		recv_data_.consume(recv_data_.size(), buffer.ptr());
	}
	else {
		buffer.regist(recv_data_.ptr(), recv_data_.size());
	}
	return true;
}

bool Net_udp::read(void* buffer, size_t size, bool consume) {
	if (!buffer)
		return false;
	if (size < 0)
		return false;

	//受信サイズが指定サイズより小さい場合は無視
	if (recv_data_.size() < size)
		return false;

	recv_data_.consume(size, buffer);
	return true;
}

bool UDP_Address::Set(string address, string port) {
	asio::io_service io_service_tmp;
	udp::resolver resolver(io_service_tmp);
	udp::resolver::query query(address, port);

	udp::resolver::iterator resolve_query;
	try {
		resolve_query = resolver.resolve(query);
	}
	catch (std::exception& e) {
		cout << e.what() << endl;
		io_service_tmp.stop();
		return false;
	}
	auto address_in = resolve_query->endpoint().address();
	auto port_in = resolve_query->endpoint().port();

	endp = udp::endpoint(address_in, port_in);

	return true;
}



void main_net_udp_test()
{
	char c;
	cout << "クライアント？サーバー？c/s\n>";
	cin >> c;
	if (c == 'c')
	{
		char ch[15];
		string server_name;
		string port_no;
		ushort my_port_no;
		cout << "サーバ名\n>";
		cin >> server_name;
		cout << "ポート番号\n>";
		cin >> port_no;
		cout << "自身のポート番号\n>";
		cin >> my_port_no;

		Net_udp client(my_port_no);
		//client.connect(server_name, port_no);

		thread t1([&] {client.run(); });

		client.join(asio::ip::address::from_string("239.255.0.1"));

		//client.start_receive();

		UDP_Address addr(server_name, port_no);
		std::this_thread::sleep_for(std::chrono::seconds(1));
		client.send("ping", addr);
		client.recv();
		client.read(ch, 6);
		cout << ch << endl;
		client.recv();
		client.read(ch, 6);
		cout << ch << endl;
		client.recv();
		client.read(ch, 6);
		cout << ch << endl;

		client.stop();
		t1.join();
	}
	else if (c == 's')
	{
		char ch[15];

		Net_udp server(51000);
		UDP_Address addr("239.255.0.1", string("60000"));

		//server.start_accept();

		thread t1([&] {server.run(); });

		cout << "start : " << addr.GetAddress().to_string() << endl;

		//server.start_receive(0);
		cout << ":::  1 -\n";
		server.recv();
		server.read(ch, 3);
		cout << ":::  2 -\n";
		server.read(&ch[3], 2);
		cout << ch << endl;

		std::this_thread::sleep_for(std::chrono::seconds(3));

		server.send("ping1", addr);
		server.send("ping2", addr);
		server.send("ping3", addr);

		server.stop();
		t1.join();

	}
}
