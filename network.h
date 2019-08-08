/*
* network.h
*
*  Created on: 2017/11/13
*      Author: c611621020
*/

#pragma once

#if _WIN32_WINNT <= _WIN32_WINNT_WINXP
#define BOOST_ASIO_ENABLE_CANCELIO
#endif

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <thread>
#include <vector>


typedef unsigned char	uchar;
typedef unsigned int	uint;
typedef unsigned short	ushort;

typedef boost::system::error_code  NetworkError;

//�o�b�t�@(�X���b�h�Z�[�t)
class Buffer {
	std::vector<uchar> data_;
	std::mutex mtx;	//�~���[�e�b�N�X
	std::atomic_int data_size;	//���ۂ̃f�[�^�T�C�Y
	std::atomic_bool dynamic_memory;	//�o�b�t�@�T�C�Y�𓮓I�ɕω������ă������e�ʂ��Œ���ɗ}����(����͕ۏ؂ł��Ȃ�����false����)
public:
	Buffer();
	Buffer(const Buffer& src);
	Buffer operator=(const Buffer& src);

	size_t size();
	size_t consume(size_t size, void* buffer = nullptr, bool start_first = true);

	//dynamic_erase : true�ɂ���ƃ������̈悩�������
	size_t clear(bool dynamic_erase = false);

	//�\���̂Ȃǂ͂���ňꔭ
	template <class T>
	size_t regist(const T& data) {
		return regist((void*)&data, sizeof(T));
	}

	//�J�X�^��
	size_t regist(void* src, size_t size);

	void* ptr();

	std::vector<uchar> data();
};

//�����񂩂�ip�A�h���X�ɕϊ�
uint addr2int(const char* str);

//ip�A�h���X���當����ɕϊ�
std::string int2addr(const uint addr);

//Buffer�N���X�͈ȉ��̃w�b�_�Ŏg���̂ł����ŃC���N���[�h
#include "network_tcp.h"
#include "network_udp.h"

