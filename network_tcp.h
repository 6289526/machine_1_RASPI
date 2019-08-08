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


//�񓯊���M�֐�
typedef std::function<void(Buffer& buf, uint recv_index, const boost::system::error_code& ec)> Recv_TCP_Func;

//�񓯊����M�֐�
typedef std::function<void(Buffer& buf, uint recv_index, const boost::system::error_code& ec)> Send_TCP_Func;

//�񓯊��ڑ��֐�
typedef std::function<void(boost::asio::ip::tcp::socket* socket, const boost::system::error_code& ec)> Connect_Func;

//�񓯊��ڑ��ҋ@�I���֐�
typedef std::function<void(boost::asio::ip::tcp::socket* socket, uint client_index, const boost::system::error_code& ec)> Accept_Func;


class Client {
protected:
	std::shared_ptr<boost::asio::io_service> io_service_ptr_; //���������Ɏw�肪�Ȃ���΍쐬 <�Q��>[http://faithandbrave.hateblo.jp/entry/20110602/1306995422]
	boost::asio::io_service& io_service_;
	std::shared_ptr<boost::asio::io_service::work> worker;

	std::atomic_bool is_connecting;	//�ڑ����Ă���Ȃ�Đڑ����Ȃ�

	boost::asio::ip::tcp::socket socket_;

	Buffer send_data_; // ���M�f�[�^
public:
	Client();

	Client(boost::asio::io_service& io_service);

	//�ڑ�
	//wait ::: �ڑ������܂ő҂�
	//connect_func ::: �ڑ���������������s����֐�
	bool connect(std::string address, std::string port, bool wait = true, Connect_Func connect_func = nullptr);

	// ����M�J�n
	void run();
	// ����M��~
	boost::system::error_code stop();

	void on_connect(const boost::system::error_code& error, Connect_Func connect_func);
	//���M�o�b�t�@�ɒǉ�
	template<class T>
	void push_sendbuf(const T& data) {
		send_data_.regist(data);
	}

	//���M�o�b�t�@�ɒǉ�(�J�X�^��)
	void push_sendbuf(const void* data, size_t size);

	// ���b�Z�[�W���M
	void send();

	// ���b�Z�[�W���M(�P��)
	void send(const void* buffer, size_t size);

	// ���b�Z�[�W���M(�P��)
	template<class T>
	void send(const T& data) {
		return send(&data, sizeof(T));
	}

	// ���M����
	// error : �G���[���
	// bytes_transferred : ���M�����o�C�g��
	void on_send(const boost::system::error_code& error);

	// ���b�Z�[�W��M
	void start_receive(size_t recv_size, Recv_TCP_Func recv_func);

	// ��M����
	// error: �G���[���
	// buff	: ��M�f�[�^���i�[����o�b�t�@
	void on_receive(const boost::system::error_code& error, std::shared_ptr<std::vector<unsigned char>> buff, Recv_TCP_Func recv_func);

	/* ��M�f�[�^���󂯎��(�f�[�^���Ȃ����false��Ԃ�)
	* buffer: ��M�f�[�^���i�[����o�b�t�@
	* size  : �󂯎��T�C�Y
	* ec	: ��M�G���[
	* �߂�l : ��M�f�[�^���Ȃ����false
	*/
	bool read_nonblock(void* buffer, size_t size, boost::system::error_code* ec = nullptr);

	/* ��M�f�[�^���󂯎��(�f�[�^���Ȃ����false��Ԃ�)
	* buffer: ��M�f�[�^���i�[����o�b�t�@
	* �߂�l : ��M�f�[�^���Ȃ����false
	*/
	template<class T>
	bool read_nonblock(const T* data) {
		return read_nonblock((void*)data, sizeof(T));
	}


	/* ��M�f�[�^���󂯎��(�󂯎��܂ő҂�)
	* buffer: ��M�f�[�^���i�[����o�b�t�@
	* size  : �󂯎��T�C�Y
	* �߂�l : ��M�G���[
	*/
	boost::system::error_code read(void* buffer, size_t size);

	// ���b�Z�[�W��M(�P��)
	template<class T>
	boost::system::error_code read(const T* data) {
		return read((void*)data, sizeof(T));
	}

	//�\�P�b�g���J���Ă��邩�H(�J���Ă����true)
	bool is_open();

	//�I�v�V������ݒ�
	template <typename SettableSocketOption>
	boost::system::error_code set_opt(const SettableSocketOption& option) {
		boost::system::error_code ec;
		socket_.set_option(option, ec);
		return ec;
	}
};

class Server {
protected:
	std::shared_ptr<boost::asio::io_service> io_service_ptr_; //���������Ɏw�肪�Ȃ���΍쐬 <�Q��>[http://faithandbrave.hateblo.jp/entry/20110602/1306995422]
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

	// ����M�J�n
	void run();
	// ����M��~
	void stop();

	// ���b�Z�[�W���M
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
	// ���M����
	// error : �G���[���
	// bytes_transferred : ���M�����o�C�g��
	void on_send(const boost::system::error_code& error);
	// ���b�Z�[�W���M(�P��)

	//acceptor�����
	void close_accept();

	//���M�o�b�t�@�ɒǉ�
	template<class T>
	void push_sendbuf(uint index, const T& data) {
		push_sendbuf(index, &data, sizeof(T));
	}

	//���M�o�b�t�@�ɒǉ�(�J�X�^��)
	void push_sendbuf(uint index, const void* data, size_t size);

	// ���b�Z�[�W���M
	void send(uint index);

	// ���b�Z�[�W���M
	void send(uint index, const void* buff, size_t size);

	// ���b�Z�[�W���M(�P��)
	template<class T>
	void send(uint index, const T& data) {
		send(index, &data, sizeof(T));
	}

	// ���b�Z�[�W��M
	void start_receive(uint index, size_t size, Recv_TCP_Func recv_func);

	// ��M����
	// error : �G���[���
	// bytes_transferred : ��M�����o�C�g��
	void on_receive(const boost::system::error_code& error, std::shared_ptr<std::vector<unsigned char>> buff, uint index, Recv_TCP_Func func);

	/* ��M�f�[�^���󂯎��(�f�[�^���Ȃ����false��Ԃ�)
	* buffer: ��M�f�[�^���i�[����o�b�t�@
	* size  : �󂯎��T�C�Y
	* ec	: ��M�G���[
	* �߂�l : ��M�f�[�^���Ȃ����false
	*/
	bool read_nonblock(uint index, void* buffer, size_t size, boost::system::error_code* ec = nullptr);

	/* ��M�f�[�^���󂯎��(�f�[�^���Ȃ����false��Ԃ�)
	* buffer: ��M�f�[�^���i�[����o�b�t�@
	* �߂�l : ��M�f�[�^���Ȃ����false
	*/
	template<class T>
	bool read_nonblock(uint index, const T* data) {
		return read_nonblock(index, (void*)data, sizeof(T));
	}

	/* ��M�f�[�^���󂯎��(�󂯎��܂ő҂�)
	* index : �Ώۂ̃N���C�A���g
	* buffer: ��M�f�[�^���i�[����o�b�t�@
	* size  : �󂯎��T�C�Y
	* �߂�l : ��M�G���[(true�Ȃ琳��)
	*/
	boost::system::error_code read(uint index, void* buffer, size_t size);

	template<class T>
	boost::system::error_code read(uint index, T* data) { return read(index, (void*)data, sizeof(T)); }

	//�\�P�b�g���J���Ă��邩�H(�J���Ă����true)
	bool is_open(uint index);

	//�I�v�V������ݒ�
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

