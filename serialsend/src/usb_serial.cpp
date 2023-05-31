#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/select.h>
#include <iostream>

#include "usb_serial.h"

using namespace std;


static int name_arr[] = { 460800, 230400, 115200, 57600, 38400,  19200,  9600,  4800,  2400,  1200,  300};

static int speed_arr[] = { B460800, B230400, B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};

USBSerial::USBSerial() {
	fd_ = -1;
}

USBSerial::~USBSerial() {
	if(fd_ != -1) {
		close(fd_);
		fd_ = -1;
	}
}

int USBSerial::Open(std::string devDesc)
{

	const char *devFile = "/dev/ttyUSB4";
	int baud = 115200;
	fd_ = open (devFile, O_RDWR | O_NOCTTY |O_NDELAY/* | O_NONBLOCK*/ );
	if (fd_ < 0)
	{
		std::cout << "error " << errno << " opening "<< devFile << std::endl;
		return -1;
	}
	else
	{

		std::cout << "success opening " << baud << devFile << std::endl;
	}
	fcntl(fd_, F_SETFL, 0);
//  fcntl(fd_, F_SETFL, FNDELAY);   // non block
	SetInterfaceAttribs (baud, 0);  // set speed to 115,200 bps, 8n1 (no parity)
//  set_blocking (0);                // set no blocking
	return fd_;
}


int USBSerial::Write(unsigned char* data, int len)
{
	int sz = (int)write(fd_, data, len);
//  tcflush(fd_, TCOFLUSH);
//  std::cout << "Send out: ";
//  for(int i=0; i<len; i++)
//      std::cout << std::hex << std::setw(2) << std::setfill('0') << (static_cast<int>(data[i]) & 0xff) <<" ";
//  std::cout << "\n";
    return sz;
}
void USBSerial::Writeandread(bool stop, int index){}

int USBSerial::Read(unsigned char* buffer, int sz)
{
	int read_len = 0;
	fd_set fs_read;
	struct timeval tv_timeout;
	tv_timeout.tv_sec = 0;
	tv_timeout.tv_usec = 0;

	FD_ZERO(&fs_read);
	FD_SET(fd_, &fs_read);
	select(fd_+1, &fs_read, NULL, NULL, &tv_timeout);

	if(FD_ISSET(fd_, &fs_read))
		read_len = read(fd_, buffer, sz);
/*
	if(read_len > 0)
	{
		for(int i=0; i<read_len; i++)
		      std::cout << std::hex << std::setw(2) << std::setfill('0') << (static_cast<int>(buffer[i]) & 0xff) <<" ";
		std::cout << "\n";
	}*/
	return read_len;
}

/*************************** Private methods **************************************/
int USBSerial::SetInterfaceAttribs (int speed, int parity)
{

	struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd_, &tty) != 0)
    {
       std::cout << "error " << errno << " from tcgetattr" << std::endl;
        return -1;
    }
    for (unsigned int i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
    	if  (speed == name_arr[i])
    	{
    		cfsetospeed (&tty, speed_arr[i]);
    		cfsetispeed (&tty, speed_arr[i]);
    		break;
    	}
    }
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input mode
    tty.c_iflag &= ~IGNBRK;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;  // raw output mode
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    cfmakeraw(&tty);
/*
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    //Make the mode to raw
    //https://stackoverflow.com/questions/15028953/sending-binary-data-over-bluetooth-rfcomm-spp-converts-0x0a-to-0x0d-0x0a
    //https://acassis.wordpress.com/2016/01/22/configuring-linuxs-serial-port-to-raw-mode/
    cfmakeraw(&tty);
*/
    if (tcsetattr (fd_, TCSANOW, &tty) != 0)
    {

       std::cout << "error " << errno << " from tcsetattr" << std::endl;;
        return -1;
    }
    return 0;
}

void USBSerial::SetBlocking (int should_block)
{

    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd_, &tty) != 0)
    {
       std::cout << "error " << errno << " from tggetattr" << std::endl;

       return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd_, TCSANOW, &tty) != 0)
    {
       std::cout << "error " << errno <<" setting term attributes" << std::endl;
       return;
    }
}


