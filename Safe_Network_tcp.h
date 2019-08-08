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

//�ڑ�
class Safe_Client : private Client
{
	std::string pair_address, pair_port;
	bool connect_clear;	//�ڑ����Ƀo�b�t�@���N���A

	Connect_Func on_connect_func;	//�����Đڑ����ɐݒ肷��֐�
public:
	Safe_Client();
	Safe_Client(boost::asio::io_service& io_service);

	~Safe_Client();

	//�ڑ�
	//wait ::: �ڑ������܂ő҂�
	//connect_func ::: �ڑ���������������s����֐�
	bool connect(const std::string address, const std::string port, bool wait = true, Connect_Func connect_func = nullptr);

	// ����M�J�n
	void run() { return Client::run(); }

	// ����M��~
	boost::system::error_code stop() { return Client::stop(); }

	void on_connect(const boost::asio::ip::tcp::socket* socket, const boost::system::error_code& error, Connect_Func connect_func);

	//���M�o�b�t�@�ɒǉ�
	template<class T>
	void push_sendbuf(const T& data) { return Client::push_sendbuf(data); }

	//���M�o�b�t�@�ɒǉ�(�J�X�^��)
	void push_sendbuf(const void* data, size_t size) { return Client::push_sendbuf(data, size); };

	// ���b�Z�[�W���M(���M��֐��t���A�o�b�t�@�����)
	void sendf(Send_TCP_Func send_func);

	// ���b�Z�[�W���M(�o�b�t�@�����)
	void send() {
		return sendf(nullptr);
	}

	// ���b�Z�[�W���M(���M��֐��t���A�P��)
	void sendf(const void* buffer, size_t size, Send_TCP_Func send_func);

	// ���b�Z�[�W���M(�P��)
	void send(const void* buffer, size_t size) {
		return sendf(buffer, size, nullptr);
	}

	// ���b�Z�[�W���M(���M��֐��t���A�P�̃e���v���[�g)
	template<class T>
	void sendf(const T& data, Send_TCP_Func send_func) { return sendf((void*)&data, sizeof(T), send_func); }

	// ���b�Z�[�W���M(�P�̃e���v���[�g)
	template<class T>
	void send(const T& data) {
		return sendf(data, nullptr);
	}

	// ���M����
	// error : �G���[���
	// bytes_transferred : ���M�����o�C�g��
	void on_send(const boost::system::error_code& error, std::shared_ptr<std::vector<unsigned char>> buff, Send_TCP_Func send_func);

	// ���b�Z�[�W��M
	void start_receive(size_t recv_size, Recv_TCP_Func recv_func);
	
	/* �o�b�t�@�ɂ��܂��Ă���T�C�Y���擾
	* ec	: �G���[
	* �߂�l : �o�b�t�@�T�C�Y / �G���[�Ȃ�-1
	*/
	int available(boost::system::error_code* ec = nullptr);

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

	//�ڑ����Ƀo�b�t�@���N���A����t���O�̐ݒ�
	void set_connect_clear(bool flag) { connect_clear = flag; };

	//�\�P�b�g���J���Ă��邩�H(�J���Ă����true)
	bool is_open() { return Client::is_open(); };

	//�I�v�V������ݒ�
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
	std::list<uint> accepting_list;	//accept���̃C���f�b�N�X���X�g

	Accept_Func on_accept_func;	//�����Đڑ����ɐݒ肷��֐�
public:
	Safe_Server();

	Safe_Server(ushort port_no);

	Safe_Server(boost::asio::io_service& io_service, ushort port_no);

	void init(ushort port_no) { return Server::init(port_no); };

	//index ::: ��t����C���f�b�N�X(���̃C���f�b�N�X�󂢂Ă��Ȃ���ΐV�����C���f�b�N�X�Ŏ�t����)
	int start_accept(bool wait = true, Accept_Func accept_func = nullptr, int index = -1);

	//acceptor�����
	void close_accept();;

	// ����M�J�n
	void run() { return Server::run(); };
	// ����M��~
	void stop() { return Server::stop(); };
	//���M�o�b�t�@�ɒǉ�
	template<class T>
	void push_sendbuf(uint index, const T& data) { return Server::push_sendbuf(index, data); };

	//���M�o�b�t�@�ɒǉ�(�J�X�^��)
	void push_sendbuf(uint index, const void* data, size_t size) { return Server::push_sendbuf(index, data, size); };

	// ���b�Z�[�W���M(���M��֐��t���A�o�b�t�@�����)
	void sendf(uint index, Send_TCP_Func send_func = nullptr);

	// ���b�Z�[�W���M(�o�b�t�@�����)
	void send(uint index) {
		return sendf(index, nullptr);
	}

	// ���b�Z�[�W���M(���M��֐��t���A�P��)
	void sendf(uint index, const void* buff, size_t size, Send_TCP_Func send_func);

	// ���b�Z�[�W���M(�P��)
	void send(uint index, const void* buff, size_t size) {
		return sendf(index, buff, size, nullptr);
	}

	// ���b�Z�[�W���M(���M��֐��t���A�P�̃e���v���[�g)
	template<class T>
	void sendf(uint index, const T& data, Send_TCP_Func send_func) {
		sendf(index, (void*)&data, sizeof(T), send_func);
	}

	// ���b�Z�[�W���M(�P�̃e���v���[�g)
	template<class T>
	void send(uint index, const T& data) {
		sendf(index, &data, sizeof(T), nullptr);
	}

	// ���M����
	// error : �G���[���
	// bytes_transferred : ���M�����o�C�g��
	void on_send(const boost::system::error_code& error, std::shared_ptr<std::vector<unsigned char>> buff, uint index, Send_TCP_Func send_func);

	// ���b�Z�[�W��M
	void start_receive(uint index, size_t size, Recv_TCP_Func recv_func);

	// ��M����
	// error : �G���[���
	// bytes_transferred : ��M�����o�C�g��
	void on_receive(const boost::system::error_code& error, std::shared_ptr<std::vector<unsigned char>> buff, uint index, Recv_TCP_Func func);

	/* �o�b�t�@�ɂ��܂��Ă���T�C�Y���擾
	* ec	: �G���[
	* �߂�l : �o�b�t�@�T�C�Y / �G���[�Ȃ�-1
	*/
	int available(uint index, boost::system::error_code* ec = nullptr);

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
	bool is_open(uint index) { return Server::is_open(index); };

	//�I�v�V������ݒ�
	template <typename SettableSocketOption>
	boost::system::error_code set_opt(uint index, const SettableSocketOption& option) {
		return set_opt(index, option);
	}

	void set_on_accept_func(Accept_Func accept_func) { on_accept_func = accept_func; };
private:
	void on_accept(const boost::system::error_code& error, uint index, Accept_Func func);
};

void main_safe_net_tcp_test();