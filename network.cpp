/*
* network.cpp
*
*  Created on: 2017/11/13
*      Author: c611621020
*/

#include "network.h"

#include <boost/asio.hpp>
#include <iostream>
#include <boost/bind.hpp>
#include <mutex>
#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include <functional>
#include <unordered_map>
#include <atomic>

using namespace std;
namespace asio = boost::asio;
using asio::ip::tcp;

//�f�o�b�O���̓l�b�g���[�N��Ԃ�\��
#ifdef __DEBUG_
#define SHOW_NETWORK_STATUS
#endif

//io_service�̏ڍׂ͂������Q��<http://yutopp.hateblo.jp/entry/2011/12/15/001518>
//��̃V���[�g��(�}���`�X���b�h�͂��������߂�)<https://qiita.com/sileader/items/74c667e51d4bb36cafab>
//�ȉ��̃N���X�̑���Ƃ͂�����<https://boostjp.github.io/tips/network/tcp.html>

Buffer::Buffer()
{
	//�f�t�H���g��False
	dynamic_memory = false;
	data_size = 0;
}

//�o�b�t�@
//�R�s�[�R���X�g���N�^
Buffer::Buffer(const Buffer& src) {
	mtx.lock();
	data_ = src.data_;
	dynamic_memory = (bool)(src.dynamic_memory);
	data_size = (int)(src.data_size);
	mtx.unlock();
}

Buffer Buffer::operator=(const Buffer& src) {
	mtx.lock();
	data_ = src.data_;
	//dynamic_memory�͈����p���Ȃ�(�񐄏��@�\�̂���)
	data_size = (int)(src.data_size);
	mtx.unlock();

	return *this;
}

size_t Buffer::consume(size_t size, void* buffer, bool start_first) {
	mtx.lock();
	if (data_size < size) {
		size = data_size;
	}
	if (buffer) {
		if (start_first)
			memcpy(buffer, &data_[0], size);
		else
			memcpy(buffer, &data_[data_.size() - size - 1], size);
	}
	//�S�ď�����ꍇ�͈ȉ��̃f�[�^���ړ����鏈���͕s�v
	if (size == data_size) {
		if (start_first) {
			memcpy(&data_[0], &data_[size], data_size - size);
		}
	}
	data_size -= size;

	mtx.unlock();
	return size;
}

size_t Buffer::clear(bool dynamic_erase) {
	mtx.lock();
	size_t size = data_size;
	data_size = 0;
	if (dynamic_erase) {
		//�f�[�^�S����
		data_.clear();
	}
	mtx.unlock();
	return size;
}

//�J�X�^��
size_t Buffer::regist(void* src, size_t size) {
	unsigned int old_size = data_size;
	mtx.lock();
	//�o�b�t�@�C�T�C�Y���������Ƃ��̂݊g��
	if(data_.size() < old_size + size)
		data_.resize(old_size + size);
	memcpy(&data_[old_size], src, size);
	data_size += size;
	mtx.unlock();
	return data_.size();
}

//�T�C�Y
size_t Buffer::size() {
	size_t tmp = data_size;
	return tmp;
}

//�f�[�^�|�C���^
void* Buffer::ptr() {
	mtx.lock();
	void* tmp = &data_[0];
	mtx.unlock();
	return tmp;
}

//�f�[�^
vector<uchar> Buffer::data() {
	mtx.lock();
	vector<uchar> tmp = data_;
	mtx.unlock();
	return tmp;
}

uint addr2int(const char * str)
{
	boost::system::error_code ec;
	asio::ip::address_v4 addr;
	addr.from_string(str, ec);

	if (ec)
		return 0;

	return (uint)addr.to_ulong();
}

string int2addr(const uint addr)
{
	asio::ip::address_v4 address(addr);

	return address.to_string();
}
