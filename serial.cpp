#include "serial.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

////////////////////////////////
//       コンストラクタ       //
////////////////////////////////
serial::serial(const char* fd_name, unsigned int baud_rate)
{
    fd = 0;
    int sci_fd;

    if(fd_name == nullptr){
        return;
    }

    this->init(fd_name, baud_rate);
}

////////////////////////////////
//        デストラクタ        //
////////////////////////////////
serial::~serial()
{
    if(fd != 0)
        this->close();
}

////////////////////////////////
// シリアルポートの初期化関数 //
////////////////////////////////
int serial::init(const char* fd_name, unsigned int baud_rate)
{
    fd = 0;
    int sci_fd;

    printf("open sci:[%s]\n",fd_name);
    if ((sci_fd = open(fd_name, O_RDWR | O_NOCTTY | O_NONBLOCK))<0)
    {
        printf("sci open failed. : %d\n", sci_fd);
        return -1;
    }
    fd = sci_fd;

    this->set_baudrate(baud_rate);

    return fd;
}

int serial::set_baudrate(unsigned int baud_rate)
{
    struct termios tio;
    int ret;

    memset(&tio,0x00,sizeof(tio));
    tio.c_cflag = baud_rate | CRTSCTS | CS8 | CLOCAL | CREAD;
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cc[VTIME] = 0;
    tio.c_cc[VMIN] = 1;

    tcflush(fd, TCIFLUSH);
    ret = tcsetattr(fd, TCSANOW, &tio);
    fcntl(fd, F_SETFL, FNDELAY);

    return ret;
}

//////////////////////////////////
// シリアルポートのクローズ関数 //
//////////////////////////////////
int serial::close()
{
        ::close(fd);
        fd = 0;
        return 0;
}

int serial::available(){
    int bytes_available = 0;
    ioctl(fd, FIONREAD, &bytes_available);

    return bytes_available;
}

/////////////////////////////
// シリアル受信関数 //
/////////////////////////////
int serial::read(void *buffer, size_t size)
{
    int retval=0;

    //サイズが負の場合は1を返す
    if(size < 0)
        return 1;

    while((retval = ::read(fd, buffer, size))<0) {
        switch (errno) {
            case EAGAIN :
            break;
            default :
                perror(strerror(errno));
                return -1;
        }
    }

    return retval;
}

///////////////////////
// シリアル送信関数 //
//////////////////////
int serial::write(void *buffer,size_t size)
{
    int retval = ::write(fd, buffer, size);
    return retval;
}