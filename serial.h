
#include <stdlib.h>
#include <termios.h>

class serial{
    int fd;
public:
    serial(const char* fd_name = nullptr, unsigned int baud_rate = B9600);
    ~serial();

    int init(const char* fd_name, unsigned int baud_rate);
    int set_baudrate(unsigned int baud_rate);
    int close();

    int available();
    int read(void *buffer, size_t size);
    int write(void *buffer, size_t size);
};