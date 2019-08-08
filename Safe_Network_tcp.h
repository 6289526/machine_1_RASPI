#pragma once

#include "network_tcp.h"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <thread>
#include <vector>
#include <list>
#include <atomic>
#include <functional>
#include <string>

//接続
class Safe_Client : private Client
{
	std::string pair_address, pair_port;
	bool connect_clear;	//接続時にバッファをクリア

	Connect_Func on_connect_func;	//自動再接続時に設定する関数
public:
	Safe_Client();
	Safe_Client(boost::asio::io_service& io_service);

	~Safe_Client();

	//接続
	//wait ::: 接続完了まで待つ
	//connect_func ::: 接続が成功したら実行する関数
	bool connect(const std::string address, const std::string port, bool wait = true, Connect_Func connect_func = nullptr);

	// 送受信開始
	void run() { return Client::run(); }

	// 送受信停止
	boost::system::error_code stop() { return Client::stop(); }

	void on_connect(const boost::asio::ip::tcp::socket* socket, const boost::system::error_code& error, Connect_Func connect_func);

	//送信バッファに追加
	template<class T>
	void push_sendbuf(const T& data) { return Client::push_sendbuf(data); }

	//送信バッファに追加(カスタム)
	void push_sendbuf(const void* data, size_t size) { return Client::push_sendbuf(data, size); };

	// メッセージ送信(送信後関数付き、バッファを解放)
	void sendf(Send_TCP_Func send_func);

	// メッセージ送信(バッファを解放)
	void send() {
		return sendf(nullptr);
	}

	// メッセージ送信(送信後関数付き、単体)
	void sendf(const void* buffer, size_t size, Send_TCP_Func send_func);

	// メッセージ送信(単体)
	void send(const void* buffer, size_t size) {
		return sendf(buffer, size, nullptr);
	}

	// メッセージ送信(送信後関数付き、単体テンプレート)
	template<class T>
	void sendf(const T& data, Send_TCP_Func send_func) { return sendf((void*)&data, sizeof(T), send_func); }

	// メッセージ送信(単体テンプレート)
	template<class T>
	void send(const T& data) {
		return sendf(data, nullptr);
	}

	// 送信完了
	// error : エラー情報
	// bytes_transferred : 送信したバイト数
	void on_send(const boost::system::error_code& error, std::shared_ptr<std::vector<unsigned char>> buff, Send_TCP_Func send_func);

	// メッセージ受信
	void start_receive(size_t recv_size, Recv_TCP_Func recv_func);
	
	/* バッファにたまっているサイズを取得
	* ec	: エラー
	* 戻り値 : バッファサイズ / エラーなら-1
	*/
	int available(boost::system::error_code* ec = nullptr);

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

	//接続時にバッファをクリアするフラグの設定
	void set_connect_clear(bool flag) { connect_clear = flag; };

	//ソケットが開いているか？(開いていればtrue)
	bool is_open() { return Client::is_open(); };

	//オプションを設定
	template <typename SettableSocketOption>
	boost::system::error_code set_opt(const SettableSocketOption& option) {
		boost::system::error_code ec;
		socket_.set_option(option, ec);
		return ec;
	}

	void set_on_connect_func(Connect_Func connect_func) { on_connect_func = connect_func; };
};

class Safe_Server : private Server
{
	std::list<uint> accepting_list;	//accept中のインデックスリスト

	Accept_Func on_accept_func;	//自動再接続時に設定する関数
public:
	Safe_Server();

	Safe_Server(ushort port_no);

	Safe_Server(boost::asio::io_service& io_service, ushort port_no);

	void init(ushort port_no) { return Server::init(port_no); };

	//index ::: 受付するインデックス(そのインデックス空いていなければ新しいインデックスで受付する)
	int start_accept(bool wait = true, Accept_Func accept_func = nullptr, int index = -1);

	//acceptorを閉じる
	void close_accept();;

	// 送受信開始
	void run() { return Server::run(); };
	// 送受信停止
	void stop() { return Server::stop(); };
	//送信バッファに追加
	template<class T>
	void push_sendbuf(uint index, const T& data) { return Server::push_sendbuf(index, data); };

	//送信バッファに追加(カスタム)
	void push_sendbuf(uint index, const void* data, size_t size) { return Server::push_sendbuf(index, data, size); };

	// メッセージ送信(送信後関数付き、バッファを解放)
	void sendf(uint index, Send_TCP_Func send_func = nullptr);

	// メッセージ送信(バッファを解放)
	void send(uint index) {
		return sendf(index, nullptr);
	}

	// メッセージ送信(送信後関数付き、単体)
	void sendf(uint index, const void* buff, size_t size, Send_TCP_Func send_func);

	// メッセージ送信(単体)
	void send(uint index, const void* buff, size_t size) {
		return sendf(index, buff, size, nullptr);
	}

	// メッセージ送信(送信後関数付き、単体テンプレート)
	template<class T>
	void sendf(uint index, const T& data, Send_TCP_Func send_func) {
		sendf(index, (void*)&data, sizeof(T), send_func);
	}

	// メッセージ送信(単体テンプレート)
	template<class T>
	void send(uint index, const T& data) {
		sendf(index, &data, sizeof(T), nullptr);
	}

	// 送信完了
	// error : エラー情報
	// bytes_transferred : 送信したバイト数
	void on_send(const boost::system::error_code& error, std::shared_ptr<std::vector<unsigned char>> buff, uint index, Send_TCP_Func send_func);

	// メッセージ受信
	void start_receive(uint index, size_t size, Recv_TCP_Func recv_func);

	// 受信完了
	// error : エラー情報
	// bytes_transferred : 受信したバイト数
	void on_receive(const boost::system::error_code& error, std::shared_ptr<std::vector<unsigned char>> buff, uint index, Recv_TCP_Func func);

	/* バッファにたまっているサイズを取得
	* ec	: エラー
	* 戻り値 : バッファサイズ / エラーなら-1
	*/
	int available(uint index, boost::system::error_code* ec = nullptr);

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
	bool is_open(uint index) { return Server::is_open(index); };

	//オプションを設定
	template <typename SettableSocketOption>
	boost::system::error_code set_opt(uint index, const SettableSocketOption& option) {
		return set_opt(index, option);
	}

	void set_on_accept_func(Accept_Func accept_func) { on_accept_func = accept_func; };
private:
	void on_accept(const boost::system::error_code& error, uint index, Accept_Func func);
};

void main_safe_net_tcp_test();