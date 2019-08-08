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

//バッファ(スレッドセーフ)
class Buffer {
	std::vector<uchar> data_;
	std::mutex mtx;	//ミューテックス
	std::atomic_int data_size;	//実際のデータサイズ
	std::atomic_bool dynamic_memory;	//バッファサイズを動的に変化させてメモリ容量を最低限に抑える(動作は保証できないためfalse推奨)
public:
	Buffer();
	Buffer(const Buffer& src);
	Buffer operator=(const Buffer& src);

	size_t size();
	size_t consume(size_t size, void* buffer = nullptr, bool start_first = true);

	//dynamic_erase : trueにするとメモリ領域からも消去
	size_t clear(bool dynamic_erase = false);

	//構造体などはこれで一発
	template <class T>
	size_t regist(const T& data) {
		return regist((void*)&data, sizeof(T));
	}

	//カスタム
	size_t regist(void* src, size_t size);

	void* ptr();

	std::vector<uchar> data();
};

//文字列からipアドレスに変換
uint addr2int(const char* str);

//ipアドレスから文字列に変換
std::string int2addr(const uint addr);

//Bufferクラスは以下のヘッダで使うのでここでインクルード
#include "network_tcp.h"
#include "network_udp.h"

