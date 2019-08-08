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
//�o�b�t�@(�X���b�h�Z�[�t)
class Buffer{
std::vector<char> data_;
std::mutex mtx;	//�~���[�e�b�N�X
public:
Buffer(){;}
Buffer(const Buffer& src);
Buffer operator=(const Buffer& src);

size_t size();
size_t consume(size_t size, void* buffer = nullptr, bool start_first = true);

size_t clear();
//�\���̂Ȃǂ͂���ňꔭ
template <class T>
size_t regist(const T& data);

//�J�X�^��
size_t regist(void* src, size_t size);

void* ptr();

std::vector<char> data();
};
*/

#define UDP_DATA_SIZE 1472 //(1500-(20+8))

//UDP�A�h���X�w���p�[�N���X
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




//�񓯊���M�֐�
typedef void(*Recv_UDP_Func)(Buffer& buf, boost::asio::ip::udp::endpoint recv_addr);

class Net_udp {
	std::shared_ptr<boost::asio::io_service> io_service_ptr_; //���������Ɏw�肪�Ȃ���΍쐬 <�Q��>[http://faithandbrave.hateblo.jp/entry/20110602/1306995422]
	boost::asio::io_service& io_service_;
	std::shared_ptr<boost::asio::io_service::work> worker;

	boost::asio::ip::udp::socket socket_;
	//std::atomic_bool recv_fin;	//��M�����m�F�p
	std::mutex read_wait_mutex;
	std::condition_variable read_mutex_;	//��M�҂��p�~���[�e�b�N�X
	std::mutex read_unique_mutex_;	//��M�����~���[�e�b�N�X
	std::mutex buffer_mutex_;	//�o�b�t�@�������݃~���[�e�b�N�X
	size_t need_size;	//��M��t�T�C�Y

	Buffer send_data_; // ���M�f�[�^
	Buffer recv_data_; //��M�o�b�t�@
public:
	Net_udp();

	Net_udp(ushort port_no);

	Net_udp(boost::asio::io_service& io_service, ushort port_no);

	void init(ushort port_num);

	//bind���ς���(�p�~)
	//void init(boost::asio::ip::udp::endpoint endp);

	//�ڑ�
	//wait ::: �ڑ������܂ő҂�
	//bool connect(std::string address, std::string port, bool wait = true);

	// ����M�J�n
	void run();
	// ����M��~
	void stop(bool ReleaseMutex = true);

	//void on_connect(const boost::system::error_code& error);
	//���M�o�b�t�@�ɒǉ�
	template<class T>
	void push_sendbuf(const T& data) {
		send_data_.regist(data);
	}

	//���M�o�b�t�@�ɒǉ�(�J�X�^��)
	void push_sendbuf(void* data, size_t size);

	// ���b�Z�[�W���M
	void send(boost::asio::ip::udp::endpoint send_endpoint);

	// ���b�Z�[�W���M(�P��)
	void send(const void* data, size_t size, boost::asio::ip::udp::endpoint send_endpoint);

	// ���b�Z�[�W���M(�P��)
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

	// ���M����
	// error : �G���[���
	// bytes_transferred : ���M�����o�C�g��
	void on_send(const boost::system::error_code& error, std::shared_ptr<Buffer> buf);
	
	// ���b�Z�[�W���M(�u���b�L���O)
	boost::system::error_code send_block(boost::asio::ip::udp::endpoint send_endpoint);
	// ���b�Z�[�W���M(�P��, �u���b�L���O)
	boost::system::error_code send_block(const void* data, size_t size, boost::asio::ip::udp::endpoint send_endpoint);

	// ���b�Z�[�W���M(�P��, �u���b�L���O)
	template<class T>
	boost::system::error_code send_block(const T& data, boost::asio::ip::udp::endpoint send_endpoint) {
		if (!socket_.is_open())
			return boost::asio::error::not_connected;

		return send_block(&data, sizeof(T), send_endpoint);
	}
	// ���b�Z�[�W��M(��M���Ɏ��s����֐�)
	void start_receive(Recv_UDP_Func recv_func);

	// ��M����
	// error : �G���[���
	// bytes_transferred : ��M�����o�C�g��
	void on_receive(const boost::system::error_code& error, std::shared_ptr<std::vector<unsigned char>> buff, boost::asio::ip::udp::endpoint endpoint, Recv_UDP_Func func);

	/* ��M�f�[�^���󂯎��(�o�b�t�@�����܂��Ă�����ǂݎ��)
	* buffer: ��M�f�[�^���i�[����o�b�t�@
	* clear : ��M�o�b�t�@��S�������ď㏑��
	* addr  : ���M���A�h���X
	* �߂�l : ��M�f�[�^�������true
	*/
	bool recv_nonblock(Buffer* buffer, bool clear = true, UDP_Address* addr = nullptr);

	/* ��M�f�[�^���󂯎��(�T�C�Y�w��,�o�b�t�@�����܂��Ă�����ǂݎ��)
	* buffer: ��M�f�[�^���i�[����o�b�t�@
	* clear : ��M�o�b�t�@��S�������ď㏑��
	* addr  : ���M���A�h���X
	* �߂�l : ��M�f�[�^�������true
	*/
	bool recvsz_nonblock(Buffer* buffer, size_t sz, bool clear = true, UDP_Address* addr = nullptr);

	/* ��M�o�b�t�@���X�V(��M����܂ő҂�)
	* clear : ��M�o�b�t�@��S�������ď㏑��
	* addr  : ���M���A�h���X
	* �߂�l : ��M�f�[�^�������true
	*/
	bool recv_nonblock(bool clear = true, UDP_Address* addr = nullptr);

	/* ��M�f�[�^���󂯎��(�󂯎��܂ő҂�)
	* buffer: ��M�f�[�^���i�[����o�b�t�@
	* clear : ��M�o�b�t�@��S�������ď㏑��
	* �߂�l : ���M���A�h���X
	*/
	UDP_Address recv(Buffer* buffer, bool clear = true);

	/* ��M�o�b�t�@���X�V(��M����܂ő҂�)
	* clear  �F ��M�o�b�t�@��S�������ď㏑��
	* �߂�l �F ���M���A�h���X
	*/
	UDP_Address recv(bool clear = true);

	/* ���ׂĂ̎�M�o�b�t�@������
	*
	*/
	bool read(Buffer& buffer, bool consume = true);

	/* ��M�o�b�t�@������
	* buffer�F �R�s�[��
	* size  �F �R�s�[�T�C�Y
	* �߂�l�F�R�s�[�\(����)�ł����true
	*/
	bool read(void* buffer, size_t size, bool consume = true);

	/* ��M�o�b�t�@������(�e���v���[�g)
	* buffer�F �R�s�[��
	* �߂�l�F�R�s�[�\(����)�ł����true
	*/
	template<class T>
	bool read(T* buffer, bool consume = true) {
		return read((void*)buffer, sizeof(T), consume);
	}

	/* �}���`�L���X�g�O���[�v��join
	*
	*/
	bool join(const boost::asio::ip::address mcast_addr) {
		boost::system::error_code ec;

		socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
		socket_.set_option(boost::asio::ip::multicast::join_group(mcast_addr), ec);

		return (bool)!ec;
	}

	/* �}���`�L���X�g�O���[�v����leave
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

	//�I�v�V������ݒ�
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
