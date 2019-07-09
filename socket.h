#pragma once


//for Windows
#if defined(_WIN32) || defined(_WIN64)

	#include <SDKDDKVer.h>

	//WindowsVista以降でないと動作しない
	//#define _WIN32_WINNT _WIN32_WINNT_VISTA 

	#if _WIN32_WINNT <= _WIN32_WINNT_WINXP
		#define BOOST_ASIO_ENABLE_CANCELIO
	#endif

#endif


#include <boost/asio.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <future>
#include <system_error>
#include <functional>

//非同期接続関数
using Connect_Func = std::function<void(boost::asio::ip::tcp::socket& socket, const boost::system::error_code& ec)>;
//非同期接続関数
using Accept_Func  = std::function<void(boost::asio::ip::tcp::socket& socket, boost::asio::ip::tcp::endpoint remote_endp, const boost::system::error_code& ec)>;

/* 再接続の種類 */
enum Connect_Type {
	ConType_Disable,	//不要
	ConType_Connect,	//Connect
	ConType_Accept,		//Accept
};
//ソケットエラー
using	  socket_error =		boost::system::error_code;
namespace socket_error_type =	boost::asio::error;


template<class Protocol>
class network_endpoint{
	boost::asio::ip::basic_endpoint<Protocol> endp;
public:
	network_endpoint() {
		set(boost::asio::ip::basic_endpoint<Protocol>());
	}
	network_endpoint(const std::string host_name, const std::string service) {
		set(host_name, service);
	}
	network_endpoint(const std::string ip_address, const unsigned short port) {
		set(ip_address, port);
	}
	network_endpoint(const boost::asio::ip::basic_endpoint<Protocol> endp) {
		set(endp);
	}

	virtual boost::system::error_code set(const std::string host_name, const std::string service) {
		boost::asio::io_context io_service;
		typename Protocol::resolver resolver(io_service);
		typename Protocol::resolver::query query(host_name, service);
		boost::system::error_code ec;


		typename Protocol::resolver::iterator resolve_query;
		typename Protocol::resolver::iterator end_itr;

		resolve_query = resolver.resolve(query, ec);
		io_service.run();
		if (ec) {
			io_service.stop();
			return ec;
		}
		
		/*
		//"localhost" == ipv4で利用
		while(resolve_query->host_name() == "localhost" && resolve_query != end_itr) {
			if (resolve_query->endpoint().address().is_v4()) {
				break;
			}
			resolve_query++;
		}
		*/

		auto address_in = resolve_query->endpoint().address();
		auto port_in = resolve_query->endpoint().port();
		endp = boost::asio::ip::basic_endpoint<Protocol>(address_in, port_in);

		io_service.stop();
		return ec;
	}

	//ipアドレス,ポート番号
	boost::system::error_code set(const std::string ip_address, const unsigned short port) {
		return set(ip_address, std::to_string(port));
	}

	template<class endpoint_type>
	boost::asio::ip::basic_endpoint<Protocol> set(const endpoint_type endpoint) {
		return endp = endpoint;
	}

	virtual boost::asio::ip::basic_endpoint<Protocol> get() {
		return endp;
	}

	//キャスト  -> Protocol::endpoint&
	operator boost::asio::ip::basic_endpoint<Protocol>&() {
		return endp;// boost::asio::ip::basic_endpoint<Protocol>(this->address(), this->port());
	}

	bool operator==(boost::asio::ip::basic_endpoint<Protocol>& src) {
		return (endp.address() == src.address()) && (endp.port() == src.port());
	}

	bool operator!=(boost::asio::ip::basic_endpoint<Protocol>& src) {
		return !(endp == src);
	}
};

//アドレス+ポート
using udp_endp = network_endpoint<boost::asio::ip::udp>;
using tcp_endp = network_endpoint<boost::asio::ip::tcp>;


template<class P, class SocketType>
class socket_base{
protected:
	//io_context
	boost::asio::io_context own_io_service;
	std::shared_ptr<boost::asio::io_context::work> work;
	std::thread service_thread;

	SocketType sock;

	//エラー時呼び出し
	virtual void error_handler(const boost::system::error_code& ec) { ; };
public:
	socket_base() : sock(own_io_service)
	{
		work = std::shared_ptr<boost::asio::io_context::work>(new boost::asio::io_context::work(own_io_service));
	};

	socket_base(boost::asio::io_context& _io_context) : sock(_io_context)
	{
		work = std::shared_ptr<boost::asio::io_context::work>(new boost::asio::io_context::work(_io_context));
	};

	socket_base(boost::asio::io_context& _io_context, boost::asio::io_context::work& _work) : sock(_io_context)
	{
		work = std::shared_ptr<boost::asio::io_context::work>(&_work);
	};

	~socket_base() {
		this->close();
	};

	//初期化(スレッド開始)
	virtual void init(std::thread* _service_thread = nullptr) {
		std::thread* th = &service_thread;
		//外部のthreadを使う場合
		if (_service_thread != nullptr)
			th = _service_thread;
		*th = std::thread([&] { sock.get_io_context().run(); });
	};

	//受信バッファに溜まっているサイズを取得
	virtual int available(boost::system::error_code* error_code) {
		boost::system::error_code ec;
		int sz = sock.available(ec);
		if (error_code)
			*error_code = ec;
		if (ec) {
			sz = -1;	//失敗している場合は-1にする
			this->error_handler(ec);	//エラーハンドラの呼び出し
		}
		return sz;
	};

	//受信
	template<class BufferType, typename CompletionCondition>
	int read_buf(BufferType& buffer, CompletionCondition completion_condition, boost::system::error_code& ec) {
		int ret = 0;

		try {
			std::future<size_t> read_size = boost::asio::async_read(sock, buffer, completion_condition, boost::asio::use_future);

			ret = read_size.get();
		}
		catch (const boost::system::system_error& error) {
			ec = error.code();
			error_handler(ec);	//エラーハンドラの呼び出し

			//一応受信できているeofエラーは受信サイズをそのまま返す
			if(ec != boost::asio::error::eof)
				ret = -1;	//エラーは-1
		}

		return ret;
	};

	//受信(サイズ指定)
	int read(void* buffer, int size, boost::system::error_code& ec) {
		static boost::asio::streambuf buf;
		auto ret = this->read_buf(buf, boost::asio::transfer_exactly(size), ec);
		std::istream stream(&buf);
		stream.read((char*)buffer, size);

		return ret;
	}

	//受信(テンプレート)
	template<class T>
	int read(T* data, boost::system::error_code& ec) {
		return this->read((char*)data, sizeof(T), ec);
	}
	/*
	//受信
	template<typename CompletionCondition>
	int read_buf(boost::asio::streambuf& buffer, CompletionCondition completion_condition) {
		boost::system::error_code ec;
		return this->read_buf(buffer, completion_condition, ec);
	};
	*/

	//受信(サイズ指定)
	int read(void* buffer, int size) {
		boost::system::error_code ec;
		return this->read(buffer, size, ec);
	}

	//受信(テンプレート)
	template<class T>
	int read(T* data) {
		return this->read(data, sizeof(T));
	}

/*-- recv_from --*/
	//受信
	template <class BufferType>
	int recv_from_buf(boost::asio::ip::basic_endpoint<P>& endp, BufferType& buffer, boost::asio::socket_base::message_flags message_flags, boost::system::error_code& ec) {
		int ret = 0;

		try {
			std::future<size_t> read_size = sock.async_receive_from(buffer, endp, message_flags, boost::asio::use_future);

			ret = read_size.get();
		}
		catch (const boost::system::system_error& error) {
			ec = error.code();
			error_handler(ec);	//エラーハンドラの呼び出し

			//一応受信できているeofエラーは受信サイズをそのまま返す
			if (ec != boost::asio::error::eof)
				ret = -1;	//エラーは-1
		}

		return ret;
	};

	//受信(サイズ指定)
	int recv_from(boost::asio::ip::basic_endpoint<P>& endp, void* buffer, int size, boost::system::error_code& ec) {
		boost::asio::mutable_buffer buf(buffer, size);
		auto ret = this->recv_from_buf(endp, buf, boost::asio::socket_base::message_flags(0) , ec);
	
		return ret;
	}

	//受信(テンプレート)
	template<class T>
	int recv_from(boost::asio::ip::basic_endpoint<P>& endp, T* data, boost::system::error_code& ec) {
		return this->recv_from(endp, (char*)data, sizeof(T), ec);
	}

	//受信(サイズ指定)
	int recv_from(boost::asio::ip::basic_endpoint<P>& endp, void* buffer, int size) {
		boost::system::error_code ec;
		return this->recv_from(endp, buffer, size, ec);
	}

	//受信(テンプレート)
	template<class T>
	int recv_from(boost::asio::ip::basic_endpoint<P>& endp, T* data) {
		return this->recv_from(endp, data, sizeof(T));
	}

	//送信
	template <class BufferType>
	int write_buf(BufferType buffer, boost::system::error_code& ec) {
		int ret = 0;

		try {
			std::future<size_t> send_size = boost::asio::async_write(sock, buffer, boost::asio::use_future);

			ret = send_size.get();
			
			//boost::asio::write(sock, buffer, ec);
		}
		catch (const boost::system::system_error& error) {
			ec = error.code();
			error_handler(ec);	//エラーハンドラの呼び出し

			ret = -1;	//エラーは-1
		}

		return ret;
	};

	//送信(サイズ指定)
	int write(void* buffer, int size, boost::system::error_code& ec) {
		return this->write_buf(boost::asio::const_buffer(buffer, size), ec);
	}

	//送信(テンプレート)
	template<class T>
	int write(T& data, boost::system::error_code& ec) {
		return this->write((void*)&data, sizeof(T), ec);
	}

	/*
	//送信
	template<class BufferType>
	int write_buf(BufferType buffer) {
		boost::system::error_code ec;
		return this->write_buf(buffer, ec);
	};
	*/

	//送信(サイズ指定)
	int write(void* buffer, int size) {
		boost::system::error_code ec;
		return this->write_buf(boost::asio::const_buffer(buffer, size), ec);
	}

	//送信(テンプレート)
	template<class T>
	int write(T& data) {
		return this->write((void*)&data, sizeof(T));
	}
	
/*-- send to (送信先指定 / UDP用)--*/
	//送信
	template <class BufferType>
	int send_buf_to(boost::asio::ip::basic_endpoint<P>& endp, BufferType buffer, boost::system::error_code& ec) {
		int ret = 0;

		try {
			std::future<size_t> send_size = sock.async_send_to(buffer, endp, boost::asio::use_future);

			ret = send_size.get();
		}
		catch (const boost::system::system_error& error) {
			ec = error.code();
			error_handler(ec);	//エラーハンドラの呼び出し

			ret = -1;	//エラーは-1
		}

		return ret;
	};

	//送信(サイズ指定)
	int send_to(boost::asio::ip::basic_endpoint<P>& endp, void* buffer, int size, boost::system::error_code& ec) {
		return this->send_buf_to(endp, boost::asio::buffer(buffer, size), ec);
	}

	//送信(テンプレート)
	template<class T>
	int send_to(boost::asio::ip::basic_endpoint<P>& endp, T& data, boost::system::error_code& ec) {
		return this->send_to(endp, (void*)&data, sizeof(T), ec);
	}

	/*
	//送信
	template<class BufferType>
	int send_buf_to(boost::asio::ip::basic_endpoint<P>& endp, BufferType buffer) {
		boost::system::error_code ec;
		return this->send_buf_to(endp, buffer, ec);
	};
	*/

	//送信(サイズ指定)
	int send_to(boost::asio::ip::basic_endpoint<P>& endp, void* buffer, int size) {
		boost::system::error_code ec;
		return this->send_buf_to(endp, boost::asio::const_buffer(buffer, size), ec);
	}

	//送信(テンプレート)
	template<class T>
	int send_to(boost::asio::ip::basic_endpoint<P>& endp, T& data) {
		return this->send_to(endp, (void*)&data, sizeof(T));
	}

	/* ソケットを閉じる
	 * stop_thread : io_contextとスレッドを終了させる
	 */
	virtual boost::system::error_code close(bool stop_thread = true) {
		boost::system::error_code ec;
		
		sock.cancel(ec);	//キャンセルしてio_contextの中身をなくす

		sock.shutdown(boost::asio::socket_base::shutdown_both, ec);
		
		sock.close(ec);		//切断

		//スレッド終了
		if (stop_thread) {
			sock.get_io_context().stop();	//io_context停止
			work.reset();	//解放してwork無効化
			if (service_thread.joinable())
				service_thread.join();	//スレッド終了
		}

		return ec;
	};

	/* オプション設定
	 */
	template <typename SettableSocketOption>
	boost::system::error_code set_option(SettableSocketOption option) {
		boost::system::error_code ec;
		sock.set_option(option, ec);

		return ec;
	}
};

class tcp_socket : public socket_base<boost::asio::ip::tcp, boost::asio::basic_stream_socket<boost::asio::ip::tcp>>{
protected:
	//acceptorは共有ポインタにすると使いやすい(https://stackoverflow.com/questions/30964017/boostasio-bind-exception-when-binding-a-endpoint)
	std::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor;

	virtual void error_handler(const boost::system::error_code& ec);

	virtual void on_connect_callback(Connect_Func connect_func, const boost::system::error_code& ec);
	virtual void on_accept_callback(Accept_Func accept_func, const boost::system::error_code& ec);
public:
	tcp_socket() :socket_base() {};
	tcp_socket(boost::asio::io_context& _io_context) :socket_base(_io_context) {};
	tcp_socket(boost::asio::io_context& _io_context, boost::asio::io_context::work& _work) :socket_base(_io_context, _work) {};

	virtual boost::system::error_code connect(boost::asio::ip::tcp::endpoint endp);
	virtual boost::system::error_code accept(unsigned short port_no, boost::asio::ip::tcp::endpoint* remote_endp = nullptr);

	virtual void connect_callback(boost::asio::ip::tcp::endpoint endp, Connect_Func connect_func = nullptr);
	virtual void accept_callback(unsigned short port_no, Accept_Func accept_func = nullptr);
};

//自動再接続機能付きTCPソケット
class tcp_safe_socket : public tcp_socket {
protected:
	std::atomic_bool is_connected;	//接続フラグ

	std::atomic_int require_connect;	//接続失敗時、再接続を行うか
	boost::asio::ip::tcp::endpoint remote_endpoint;		//再接続先(accept時は自ポート)

	//接続後の処理(callback関数)
	Accept_Func  on_accept_func = nullptr;
	Connect_Func on_connect_func= nullptr;

	virtual void error_handler(const boost::system::error_code& ec);
	
	//**_callbackで接続後(失敗時も)に呼び出される関数
	virtual void on_connect_callback(Connect_Func connect_func, const boost::system::error_code& ec);
	virtual void on_accept_callback(Accept_Func accept_func, const boost::system::error_code& ec);
public:
	tcp_safe_socket() : tcp_socket() { is_connected = false; require_connect = ConType_Disable; };
	tcp_safe_socket(boost::asio::io_context& _io_context) : tcp_socket(_io_context) { is_connected = false; require_connect = ConType_Disable; };
	tcp_safe_socket(boost::asio::io_context& _io_context, boost::asio::io_context::work& _work) : tcp_socket(_io_context, _work) { is_connected = false; require_connect = ConType_Disable; };

	virtual boost::system::error_code connect(boost::asio::ip::tcp::endpoint endp);
	virtual boost::system::error_code accept(unsigned short port_no, boost::asio::ip::tcp::endpoint* remote_endp = nullptr);

	virtual void connect_callback(boost::asio::ip::tcp::endpoint endp, Connect_Func connect_func = nullptr);
	virtual void accept_callback(unsigned short port_no, Accept_Func accept_func = nullptr);

	bool is_connect() { return is_connected; }
	//再接続
	void reconnect(Accept_Func accept_func = nullptr, Connect_Func connect_func = nullptr);

	Connect_Type get_connect_type() { return (Connect_Type)require_connect.load(); }
	virtual boost::system::error_code close(bool stop_thread = true);
};

//UDPソケット
class udp_socket : public socket_base<boost::asio::ip::udp, boost::asio::basic_datagram_socket<boost::asio::ip::udp>>{
protected:
	std::atomic_bool is_connected;

	virtual void error_handler(const boost::system::error_code& ec);
public:
	udp_socket() :socket_base() { is_connected = false; };
	udp_socket(boost::asio::io_context& _io_context) :socket_base(_io_context) { is_connected = false; };
	udp_socket(boost::asio::io_context& _io_context, boost::asio::io_context::work& _work) :socket_base(_io_context, _work) { is_connected = false; };

	boost::system::error_code bind(unsigned short port_no);

	boost::system::error_code join(udp_endp multicast_endp);
	boost::system::error_code leave(udp_endp multicast_endp);
};
