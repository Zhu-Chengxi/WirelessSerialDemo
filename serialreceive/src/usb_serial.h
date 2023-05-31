#ifndef USB_SERIAL_H
#define USB_SERIAL_H

#include <boost/shared_ptr.hpp>



class USBSerial
{
private:
	int fd_;

	int SetInterfaceAttribs (int speed, int parity);
	void SetBlocking (int should_block);

public:
	USBSerial();
	~USBSerial();
	int Open(std::string devDesc);
	int Write(unsigned char* data, int len);
	int Read(unsigned char* buffer, int sz);
	void Writeandread(bool stop, int index);

};


#endif //USB_SERIAL_H
