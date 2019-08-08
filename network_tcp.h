/*
* network.h
*
*  Created on: 2017/11/13
*      Author: c611621020
*/

#pragma once

#include "network.h"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <thread>
#include <vector>
#include <functional>
#include <atomic>


//非同期受信関数
typedef std::function<void(Buffer& buf, uint recv_index, const boost::system::error_code& ec)> Recv_TCP_Func;

//非同期送信関数
typedef std::function<void(Buffer& buf, uint recv_index, const boost::system::error_code& ec)> Send_TCP_Func;

//非同期接続関数
typedef std::function<void(boost::asio::ip::tcp::socket* socket, const boost::system::error_code& ec)> Connect_Func;

//非同期接続待機終了関数
typedef std::function<void(boost::asio::ip::tcp::socket* socket, uint client_index, const boost::system::error_code& ec)> Accept_Func;


class Client {
protected:
	std::shared_ptr<boost::asio::io_service> io_service_ptr_; //初期化時に指定がなければ作成 <参照>[http://faithandbrave.hateblo.jp/entry/20110602/1306995422]
	boost::asio::io_service& io_service_;
	std::shared_ptr<boost::asio::io_service::work> worker;

	std::atomic_bool is_connecting;	//接続しているなら再接続しない

	boost::asio::ip::tcp::socket socket_;

	Buffer send_data_; // 送信データ
public:
	Client();

	Client(boost::asio::io_service& io_service);

	//接続
	//wait ::: 接続完了まで待つ
	//connect_func ::: 接続が成功したら実行する関数
	bool connect(std::string address, std::string port, bool wait = true, Connect_Func connect_func = nullptr);

	// 送受信開始
	void run();
	// 送受信停止
	boost::system::error_code stop();

	void on_connect(const boost::system::error_code& error, Connect_Func connect_func);
	//送信バッファに追加
	template<class T>
	void push_sendbuf(const T& data) {
		send_data_.regist(data);
	}

	//送信バッファに追加(カスタム)
	void push_sendbuf(const void* data, size_t size);

	// メッセージ送信
	void send();

	// メッセージ送信(単体)
	void send(const void* buffer, size_t size);

	// メッセージ送信(単体)
	template<class T>
	void send(const T& data) {
		return send(&data, sizeof(T));
	}

	// 送信完了
	// error : エラー情報
	// bytes_transferred : 送信したバイト数
	void on_send(const boost::system::error_code& error);

	// メッセージ受信
	void start_receive(size_t recv_size, Recv_TCP_Func recv_func);

	// 受信完了
	// error: エラー情報
	// buff	: 受信データを格納するバッファ
	void on_receive(const boost::system::error_code& error, std::shared_ptr<std::vector<unsigned char>> buff, Recv_TCP_Func recv_func);

	/* 受信データを受け取る(データがなければfalseを返す)
	* buffer: 受信データを格納するバッファ
	* size  : 受け取るサイズ
	* ec	: 受信エラー
	* 戻り値 : 受信データがなければfalse
	*/
	bool read_nonblock(void* buffer, size_t size, boost::system::error_code* ec = nullptr);

	/* 受信データを受け取る(データがなければfalseを返す)
	* buffer: 受信データを格納するバッファ
	* 戻り値 : 受信データがなければfalse
	*/
	template<class T>
	bool read_nonblock(const T* data) {
		return read_nonblock((void*)data, sizeof(T));
	}


	/* 受信データを受け取る(受け取るまで待つ)
	* buffer: 受信データを格納するバッファ
	* size  : 受け取るサイズ
	* 戻り値 : 受信エラー
	*/
	boost::system::error_code read(void* buffer, size_t size);

	// メッセージ受信(単体)
	template<class T>
	boost::system::error_code read(const T* data) {
		return read((void*)data, sizeof(T));
	}

	//ソケットが開いているか？(開いていればtrue)
	bool is_open();

	//オプションを設定
	template <typename SettableSocketOption>
	boost::system::error_code set_opt(const SettableSocketOption& option) {
		boost::system::error_code ec;
		socket_.set_option(option, ec);
		return ec;
	}
};

class Server {
protected:
	std::shared_ptr<boost::asio::io_service> io_service_ptr_; //初期化時に指定がなければ作成 <参照>[http://faithandbrave.hateblo.jp/entry/20110602/1306995422]
	boost::asio::io_service& io_service_;
	std::shared_ptr<boost::asio::io_service::work> worker;

	boost::asio::ip::tcp::acceptor acceptor_;
	std::vector<std::shared_ptr<boost::asio::ip::tcp::socket>> socket_;
	std::vector<Buffer> send_data_;

public:
	Server();

	Server(ushort port_no);

	Server(boost::asio::io_service& io_service, ushort port_no);

	void init(ushort port_no);

	int start_accept(bool wait = true, Accept_Func accept_func = nullptr);

	// 送受信開始
	void run();
	// 送受信停止
	void stop();

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
	void on_send(const boost::system::error_code& error);
	// メッセージ送信(単体)

	//acceptorを閉じる
	void close_accept();

	//送信バッファに追加
	template<class T>
	void push_sendbuf(uint index, const T& data) {
		push_sendbuf(index, &data, sizeof(T));
	}

	//送信バッファに追加(カスタム)
	void push_sendbuf(uint index, const void* data, size_t size);

	// メッセージ送信
	void send(uint index);

	// メッセージ送信
	void send(uint index, const void* buff, size_t size);

	// メッセージ送信(単体)
	template<class T>
	void send(uint index, const T& data) {
		send(index, &data, sizeof(T));
	}

	// メッセージ受信
	void start_receive(uint index, size_t size, Recv_TCP_Func recv_func);

	// 受信完了
	// error : エラー情報
	// bytes_transferred : 受信したバイト数
	void on_receive(const boost::system::error_code& error, std::shared_ptr<std::vector<unsigned char>> buff, uint index, Recv_TCP_Func func);

	/* 受信データを受け取る(データがなければfalseを返す)
	* buffer: 受信データを格納するバッファ
	* size  : 受け取るサイズ
	* ec	: 受信エラー
	* 戻り値 : 受信データがなければfalse
	*/
	bool read_nonblock(uint index, void* buffer, size_t size, boost::system::error_code* ec = nullptr);

	/* 受信データを受け取る(データがなければfalseを返す)
	* buffer: 受信データを格納するバッファ
	* 戻り値 : 受信データがなければfalse
	*/
	template<class T>
	bool read_nonblock(uint index, const T* data) {
		return read_nonblock(index, (void*)data, sizeof(T));
	}

	/* 受信データを受け取る(受け取るまで待つ)
	* index : 対象のクライアント
	* buffer: 受信データを格納するバッファ
	* size  : 受け取るサイズ
	* 戻り値 : 受信エラー(trueなら正常)
	*/
	boost::system::error_code read(uint index, void* buffer, size_t size);

	template<class T>
	boost::system::error_code read(uint index, T* data) { return read(index, (void*)data, sizeof(T)); }

	//ソケットが開いているか？(開いていればtrue)
	bool is_open(uint index);

	//オプションを設定
	template <typename SettableSocketOption>
	boost::system::error_code set_opt(uint index, const SettableSocketOption& option) {
		if (socket_.size() <= index)
			return boost::asio::error::invalid_argument;
		boost::system::error_code ec;
		socket_[index]->set_option(option, ec);
		return ec;
	}
private:
	void on_accept(const boost::system::error_code& error, uint index, Accept_Func func);
};

