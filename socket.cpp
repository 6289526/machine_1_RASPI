#include "socket.h"

#include <boost/bind.hpp>

//#define DEBUG_MESSAGE_WIN

using namespace std;
namespace asio = boost::asio;
using boost::asio::ip::tcp;
using boost::asio::ip::udp;

void tcp_socket::error_handler(const boost::system::error_code & ec){
	switch (ec.value()) {
	case boost::asio::error::connection_refused:	//接続失敗
		break;
	case boost::asio::error::not_connected:			//未接続状態
		break;
	}
#ifdef DEBUG_MESSAGE_WIN
	MessageBox(0, ec.message().c_str(), to_string(ec.value()).c_str(), 0);
#endif
}

void tcp_socket::on_connect_callback(Connect_Func connect_func, const boost::system::error_code & ec){
	
	if (ec) {
		//connect_funcがなければerror_handleを呼び出す
		if (connect_func != nullptr) {
			connect_func(this->sock, ec);
		}
		else {
			error_handler(ec);
		}
	}
	else {
		if(connect_func != nullptr)
			connect_func(this->sock, ec);
	}
}

void tcp_socket::on_accept_callback(Accept_Func accept_func, const boost::system::error_code & ec){
	
	if (ec) {
		//connect_funcがなければerror_handleを呼び出す
		if (accept_func != nullptr) {
			accept_func(this->sock, tcp::endpoint(), ec);
		}
		else {
			error_handler(ec);
		}
	}
	else {
		boost::system::error_code endp_ec;
		//endp_ecで例外処理を飛ばす
		boost::asio::ip::tcp::endpoint remote_endp = sock.remote_endpoint(endp_ec);

		if (accept_func != nullptr)
			accept_func(this->sock, remote_endp, ec);
	}

	//acceptorを閉じる
	boost::system::error_code ec_acceptor;
	acceptor->cancel(ec_acceptor);
	acceptor->close(ec_acceptor);
}

boost::system::error_code tcp_socket::connect(boost::asio::ip::tcp::endpoint endp)
{
	boost::system::error_code ec;
	try {
		
		future<void> f = sock.async_connect(endp, boost::asio::use_future);

		f.get();	//getで実行(エラーなら例外呼び出し)
	}
	catch (const boost::system::system_error& error) {
		ec = error.code();
		error_handler(ec);	//エラーハンドラの呼び出し
	}

	return ec;
}

boost::system::error_code tcp_socket::accept(unsigned short port_no, tcp::endpoint* remote_endp)
{
	boost::system::error_code ec;
	tcp::endpoint rem_endp;
	//tcp::acceptor acceptor(sock.get_io_context(), tcp::endpoint(tcp::v4(), port_no));
	
	try {
		acceptor.reset(new boost::asio::ip::tcp::acceptor(sock.get_io_context(), boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port_no)));

		future<void> f = acceptor->async_accept(sock, boost::asio::use_future);

		f.get();	//getで実行(エラーなら例外呼び出し)

		rem_endp = tcp::endpoint(sock.remote_endpoint().address(), sock.remote_endpoint().port());

		if (remote_endp)
			*remote_endp = rem_endp;
	}
	catch (const boost::system::system_error& error) {
		ec = error.code();
		error_handler(ec);	//エラーハンドラの呼び出し
	}

	//acceptorを閉じる
	boost::system::error_code ec_acceptor;
	acceptor->cancel(ec_acceptor);
	acceptor->close(ec_acceptor);

	return ec;
}

void tcp_socket::connect_callback(boost::asio::ip::tcp::endpoint endp, Connect_Func connect_func){
	sock.async_connect(endp,
		boost::bind(&tcp_socket::on_connect_callback,
			this,
			connect_func,
			asio::placeholders::error
		)
	);
}

void tcp_socket::accept_callback(unsigned short port_no, Accept_Func accept_func){
	acceptor.reset(new boost::asio::ip::tcp::acceptor(sock.get_io_context(), boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port_no)));

	acceptor->async_accept(
		sock,
		boost::bind(&tcp_socket::on_accept_callback,
			this,
			accept_func,
			asio::placeholders::error
		)
	);
}

void udp_socket::error_handler(const boost::system::error_code & ec) {
	switch (ec.value()) {
	case boost::asio::error::connection_refused:	//接続失敗
		break;
	case boost::asio::error::not_connected:			//未接続状態
		break;
	}
#ifdef DEBUG_MESSAGE_WIN
	MessageBox(0, ec.message().c_str(), to_string(ec.value()).c_str(), 0);
#endif
}

boost::system::error_code udp_socket::bind(unsigned short port_no)
{
	boost::system::error_code ec;

	sock.open(udp::v4(),ec);
	if(ec){
		error_handler(ec);	//エラーハンドラの呼び出し
	}

	sock.bind(udp::endpoint(udp::v4(), port_no), ec);

	if(ec){
		error_handler(ec);	//エラーハンドラの呼び出し
	}

	return ec;
}

boost::system::error_code udp_socket::join(udp_endp multicast_endp)
{
	boost::system::error_code ec;
	ec = this->set_option(udp::socket::reuse_address(true));
	ec = this->set_option(asio::ip::multicast::join_group(multicast_endp.get().address()));

	return ec;
}

boost::system::error_code udp_socket::leave(udp_endp multicast_endp)
{
	boost::system::error_code ec;
	ec = this->set_option(asio::ip::multicast::leave_group(multicast_endp.get().address()));

	return ec;
}

void tcp_safe_socket::error_handler(const boost::system::error_code & ec)
{
	switch (ec.value()) {
	case boost::asio::error::eof:	//(not Error)
		return;
	default:
		//if (is_connected) {
			//何らかのエラーが発生したら直ちに再接続
			this->reconnect(on_accept_func, on_connect_func);
		//}
	}
}

void tcp_safe_socket::on_connect_callback(Connect_Func connect_func, const boost::system::error_code & ec){
	tcp_socket::on_connect_callback(connect_func, ec);

	//接続成功
	if (!ec) {
		is_connected = true;
	}
}

void tcp_safe_socket::on_accept_callback(Accept_Func accept_func, const boost::system::error_code & ec){
	tcp_socket::on_accept_callback(accept_func, ec);

	//接続成功
	if (!ec) {
		is_connected = true;
	}
}

boost::system::error_code tcp_safe_socket::connect(boost::asio::ip::tcp::endpoint endp)
{
	boost::system::error_code ec;

	//いったん切断しておく(require_connect等が書き換わるのでこの後再設定が必要)
	close(false);
	//自動再接続機能ON(Connect)
	require_connect = ConType_Connect;
	//接続先を記憶
	remote_endpoint = endp;
	//接続関数を無効化
	on_connect_func = nullptr;

	ec = tcp_socket::connect(endp);

	//接続成功
	if (!ec) {
		is_connected = true;
	}

	return ec;
}

boost::system::error_code tcp_safe_socket::accept(unsigned short port_no, boost::asio::ip::tcp::endpoint * remote_endp)
{
	boost::system::error_code ec;

	//いったん切断しておく(require_connect等が書き換わるのでこの後再設定が必要)
	close(false);
	//自動再接続機能ON(Accept)
	require_connect = ConType_Accept;
	//ポート番号を記憶
	remote_endpoint = tcp::endpoint(tcp::v4(), port_no);
	//接続関数を無効化
	on_accept_func = nullptr;

	ec = tcp_socket::accept(port_no, remote_endp);

	//接続成功
	if (!ec) {
		is_connected = true;
	}

	return ec;
}

void tcp_safe_socket::connect_callback(boost::asio::ip::tcp::endpoint endp, Connect_Func connect_func){
	//いったん切断しておく(require_connect等が書き換わるのでこの後再設定が必要)
	close(false);
	//自動再接続機能ON(Connect)
	require_connect = ConType_Connect;
	//接続先を記憶
	remote_endpoint = endp;
	//接続関数を設定
	on_connect_func = connect_func;
	
	sock.async_connect(endp,
		boost::bind(&tcp_safe_socket::on_connect_callback,
			this,
			connect_func,
			asio::placeholders::error
		)
	);
}

void tcp_safe_socket::accept_callback(unsigned short port_no, Accept_Func accept_func){
	//いったん切断しておく(require_connect等が書き換わるのでこの後再設定が必要)
	close(false);
	//自動再接続機能ON(Accept)
	require_connect = ConType_Accept;
	//ポート番号を記憶
	remote_endpoint = tcp::endpoint(tcp::v4(), port_no);
	//接続関数を設定
	on_accept_func = accept_func;
	
	acceptor.reset(new boost::asio::ip::tcp::acceptor(sock.get_io_context(), boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port_no)));

	acceptor->async_accept(
		sock,
		boost::bind(&tcp_safe_socket::on_accept_callback,
			this,
			accept_func,
			asio::placeholders::error
		)
	);
}

void tcp_safe_socket::reconnect(Accept_Func accept_func, Connect_Func connect_func)
{
	//再接続
	switch (require_connect) {
	case ConType_Accept:
		accept_callback(remote_endpoint.port(), accept_func);
		break;
	case ConType_Connect:
		connect_callback(remote_endpoint, connect_func);
		break;
	default:
		break;
	}
}

boost::system::error_code tcp_safe_socket::close(bool stop_thread)
{
	boost::system::error_code ec;

	is_connected = false;
	//自動再接続機能OFF
	require_connect = ConType_Disable;

	ec = tcp_socket::close(stop_thread);
	
	return ec;
}
