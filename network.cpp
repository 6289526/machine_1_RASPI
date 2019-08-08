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

//デバッグ時はネットワーク状態を表示
#ifdef __DEBUG_
#define SHOW_NETWORK_STATUS
#endif

//io_serviceの詳細はここを参照<http://yutopp.hateblo.jp/entry/2011/12/15/001518>
//上のショート版(マルチスレッドはこっちが近い)<https://qiita.com/sileader/items/74c667e51d4bb36cafab>
//以下のクラスの大もとはこっち<https://boostjp.github.io/tips/network/tcp.html>

Buffer::Buffer()
{
	//デフォルトでFalse
	dynamic_memory = false;
	data_size = 0;
}

//バッファ
//コピーコンストラクタ
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
	//dynamic_memoryは引き継がない(非推奨機能のため)
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
	//全て消費した場合は以下のデータを移動する処理は不要
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
		//データ全消去
		data_.clear();
	}
	mtx.unlock();
	return size;
}

//カスタム
size_t Buffer::regist(void* src, size_t size) {
	unsigned int old_size = data_size;
	mtx.lock();
	//バッファイサイズが小さいときのみ拡張
	if(data_.size() < old_size + size)
		data_.resize(old_size + size);
	memcpy(&data_[old_size], src, size);
	data_size += size;
	mtx.unlock();
	return data_.size();
}

//サイズ
size_t Buffer::size() {
	size_t tmp = data_size;
	return tmp;
}

//データポインタ
void* Buffer::ptr() {
	mtx.lock();
	void* tmp = &data_[0];
	mtx.unlock();
	return tmp;
}

//データ
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
