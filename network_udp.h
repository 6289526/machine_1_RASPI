/*
* network.h
*
*  Created on: 2017/11/13
*      Author: c611621020
*/

#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include "network.h"
/*
//バッファ(スレッドセーフ)
class Buffer{
std::vector<char> data_;
std::mutex mtx;	//ミューテックス
public:
Buffer(){;}
Buffer(const Buffer& src);
Buffer operator=(const Buffer& src);

size_t size();
size_t consume(size_t size, void* buffer = nullptr, bool start_first = true);

size_t clear();
//構造体などはこれで一発
template <class T>
size_t regist(const T& data);

//カスタム
size_t regist(void* src, size_t size);

void* ptr();

std::vector<char> data();
};
*/

#define UDP_DATA_SIZE 1472 //(1500-(20+8))

//UDPアドレスヘルパークラス
class UDP_Address {
	boost::asio::ip::udp::endpoint endp;

public:
	UDP_Address() { ; };
	UDP_Address(const UDP_Address& src) { endp = src.endp; };
	UDP_Address(std::string addr, std::string port) { Set(addr, port); };

	bool Set(std::string addr, std::string port);

	boost::asio::ip::address GetAddress() { return endp.address(); };
	unsigned short GetPort() { return endp.port(); };

	boost::asio::ip::udp::endpoint operator=(const boost::asio::ip::udp::endpoint& src) { return endp = src; };
	operator boost::asio::ip::udp::endpoint() const { return endp; };

	bool operator ==(UDP_Address& addr) {
		return ((endp.address() == addr.GetAddress()) && (endp.port() == addr.GetPort()));
	}
};




//非同期受信関数
typedef void(*Recv_UDP_Func)(Buffer& buf, boost::asio::ip::udp::endpoint recv_addr);

class Net_udp {
	std::shared_ptr<boost::asio::io_service> io_service_ptr_; //初期化時に指定がなければ作成 <参照>[http://faithandbrave.hateblo.jp/entry/20110602/1306995422]
	boost::asio::io_service& io_service_;
	std::shared_ptr<boost::asio::io_service::work> worker;

	boost::asio::ip::udp::socket socket_;
	//std::atomic_bool recv_fin;	//受信完了確認用
	std::mutex read_wait_mutex;
	std::condition_variable read_mutex_;	//受信待ち用ミューテックス
	std::mutex read_unique_mutex_;	//受信競合ミューテックス
	std::mutex buffer_mutex_;	//バッファ書き込みミューテックス
	size_t need_size;	//受信受付サイズ

	Buffer send_data_; // 送信データ
	Buffer recv_data_; //受信バッファ
public:
	Net_udp();

	Net_udp(ushort port_no);

	Net_udp(boost::asio::io_service& io_service, ushort port_no);

	void init(ushort port_num);

	//bind先を変える(廃止)
	//void init(boost::asio::ip::udp::endpoint endp);

	//接続
	//wait ::: 接続完了まで待つ
	//bool connect(std::string address, std::string port, bool wait = true);

	// 送受信開始
	void run();
	// 送受信停止
	void stop(bool ReleaseMutex = true);

	//void on_connect(const boost::system::error_code& error);
	//送信バッファに追加
	template<class T>
	void push_sendbuf(const T& data) {
		send_data_.regist(data);
	}

	//送信バッファに追加(カスタム)
	void push_sendbuf(void* data, size_t size);

	// メッセージ送信
	void send(boost::asio::ip::udp::endpoint send_endpoint);

	// メッセージ送信(単体)
	void send(const void* data, size_t size, boost::asio::ip::udp::endpoint send_endpoint);

	// メッセージ送信(単体)
	template<class T>
	void send(const T& data, boost::asio::ip::udp::endpoint send_endpoint) {
		if (!socket_.is_open())
			return;

		return send(&data, sizeof(T), send_endpoint);

		/*
		boost::asio::async_write(
		socket_,
		boost::asio::buffer(buf->data()),
		boost::bind(&Net_udp::on_send, this,
		boost::asio::placeholders::error,
		buf));
		*/
	}

	// 送信完了
	// error : エラー情報
	// bytes_transferred : 送信したバイト数
	void on_send(const boost::system::error_code& error, std::shared_ptr<Buffer> buf);
	
	// メッセージ送信(ブロッキング)
	boost::system::error_code send_block(boost::asio::ip::udp::endpoint send_endpoint);
	// メッセージ送信(単体, ブロッキング)
	boost::system::error_code send_block(const void* data, size_t size, boost::asio::ip::udp::endpoint send_endpoint);

	// メッセージ送信(単体, ブロッキング)
	template<class T>
	boost::system::error_code send_block(const T& data, boost::asio::ip::udp::endpoint send_endpoint) {
		if (!socket_.is_open())
			return boost::asio::error::not_connected;

		return send_block(&data, sizeof(T), send_endpoint);
	}
	// メッセージ受信(受信時に実行する関数)
	void start_receive(Recv_UDP_Func recv_func);

	// 受信完了
	// error : エラー情報
	// bytes_transferred : 受信したバイト数
	void on_receive(const boost::system::error_code& error, std::shared_ptr<std::vector<unsigned char>> buff, boost::asio::ip::udp::endpoint endpoint, Recv_UDP_Func func);

	/* 受信データを受け取る(バッファが溜まっていたら読み取る)
	* buffer: 受信データを格納するバッファ
	* clear : 受信バッファを全消去して上書き
	* addr  : 送信元アドレス
	* 戻り値 : 受信データがあればtrue
	*/
	bool recv_nonblock(Buffer* buffer, bool clear = true, UDP_Address* addr = nullptr);

	/* 受信データを受け取る(サイズ指定,バッファが溜まっていたら読み取る)
	* buffer: 受信データを格納するバッファ
	* clear : 受信バッファを全消去して上書き
	* addr  : 送信元アドレス
	* 戻り値 : 受信データがあればtrue
	*/
	bool recvsz_nonblock(Buffer* buffer, size_t sz, bool clear = true, UDP_Address* addr = nullptr);

	/* 受信バッファを更新(受信するまで待つ)
	* clear : 受信バッファを全消去して上書き
	* addr  : 送信元アドレス
	* 戻り値 : 受信データがあればtrue
	*/
	bool recv_nonblock(bool clear = true, UDP_Address* addr = nullptr);

	/* 受信データを受け取る(受け取るまで待つ)
	* buffer: 受信データを格納するバッファ
	* clear : 受信バッファを全消去して上書き
	* 戻り値 : 送信元アドレス
	*/
	UDP_Address recv(Buffer* buffer, bool clear = true);

	/* 受信バッファを更新(受信するまで待つ)
	* clear  ： 受信バッファを全消去して上書き
	* 戻り値 ： 送信元アドレス
	*/
	UDP_Address recv(bool clear = true);

	/* すべての受信バッファを消費
	*
	*/
	bool read(Buffer& buffer, bool consume = true);

	/* 受信バッファを消費
	* buffer： コピー先
	* size  ： コピーサイズ
	* 戻り値：コピー可能(完了)であればtrue
	*/
	bool read(void* buffer, size_t size, bool consume = true);

	/* 受信バッファを消費(テンプレート)
	* buffer： コピー先
	* 戻り値：コピー可能(完了)であればtrue
	*/
	template<class T>
	bool read(T* buffer, bool consume = true) {
		return read((void*)buffer, sizeof(T), consume);
	}

	/* マルチキャストグループにjoin
	*
	*/
	bool join(const boost::asio::ip::address mcast_addr) {
		boost::system::error_code ec;

		socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
		socket_.set_option(boost::asio::ip::multicast::join_group(mcast_addr), ec);

		return (bool)!ec;
	}

	/* マルチキャストグループからleave
	*
	*/
	bool leave(const boost::asio::ip::address mcast_addr) {
		boost::system::error_code ec;

		socket_.set_option(boost::asio::ip::multicast::leave_group(mcast_addr), ec);

		return (bool)!ec;
	}

	size_t size() {
		return recv_data_.size();
	}

	//オプションを設定
	template <typename SettableSocketOption>
	boost::system::error_code set_opt(const SettableSocketOption& option) {
		boost::system::error_code ec;
		socket_.set_option(option, ec);
		return ec;
	}

	bool is_open() {
		return socket_.is_open();
	}
};
