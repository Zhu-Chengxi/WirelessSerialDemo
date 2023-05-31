/*
 * robot_connector.cc
 *
 *  Created on: Apr 24, 2018
 *      Author: zbmai
 */



#include "usb_serial.h"

#include <iostream>
#include <thread>
#include <stdlib.h> 
#include <stdio.h>        // C标准库用来调用printf
#include <sys/time.h>    //  gettimeofday()的头文件
#include <unistd.h>        // 加入延迟更加方便我们观察
#include <iomanip>
#include <time.h>
#include <chrono>  
#include <iostream>   // std::cout
#include <math.h>


unsigned char last_rec;
unsigned char cur_rec;
int start_flag = 0;
int recv_cnt = 0;
int recv_data_len_ = 10;
unsigned char recv_data[64];
USBSerial serial;
long int timess = 0;
void RecFun();
bool ReceiveData( );
int Read( unsigned char* buffer, int sz);
int Write(unsigned char* data, int len);
void ReadInitsensors();

std::time_t getTimeStamp()
{
    std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());//获取当前时间点
    std::time_t timestamp =  tp.time_since_epoch().count(); //计算距离1970-1-1,00:00的时间长度
    return timestamp;
}

void RecFun(){
	while(true){
		if(ReceiveData()){
		//	std::cout << "recive data: " << std::hex << (int)recv_data[0] << ", " << (int)recv_data[1] << ", " << (int)recv_data[2] << ", " << (int)recv_data[3] << ", "
		//				<< std::hex << (int)recv_data[4] << ", " << (int)recv_data[5] << ", " << (int)recv_data[6] << ", " << (int)recv_data[7] << ", "
		//				<< std::hex << (int)recv_data[8] << ", " << (int)recv_data[9] << std::endl;
			time_t timestamp;
			timestamp =  getTimeStamp();
			timess = static_cast<long int>(timestamp);
			//usleep(10000); // 10ms
		}
	}
}

bool ReceiveData( ) {


	unsigned char raw_buf[1];
	
	int nread = Read(raw_buf, 1);
	
	unsigned char ch;


	if(nread == 1)
		//std::cout << "nread = " << nread << ", " <<  std::hex << (int)raw_buf[0] << std::endl;
		//std::cout << "recv_data[" << recv_cnt << "] = " << std::hex << (int)recv_data[recv_cnt] << std::endl;
		//std::cout << recv_cnt << std::endl;
	for(int i=0; i < nread; i++){
		last_rec = cur_rec;
		cur_rec = raw_buf[i];
		if(last_rec == 0xee && cur_rec ==0xaa)
		{
			recv_data[0] = 0xee;
			recv_data[1] = 0xaa;
			recv_cnt = 1;
		}
		
		recv_data[recv_cnt] = cur_rec;
		recv_cnt++;
		time_t timestamp;
		timestamp =  getTimeStamp();
	
		long int t = static_cast<long int>(timestamp);
			
		if(recv_cnt == 15 && cur_rec == 0xbb)
		{
			std::cout << std::dec << t << "   :::::::" << (int)recv_data[0] << ", " << (int)recv_data[1] << ", " << (int)recv_data[2] << ", " << (int)recv_data[3] << ", "
						<< std::hex << (int)recv_data[4] << ", " << (int)recv_data[5] << ", " << (int)recv_data[6] << ", " << (int)recv_data[7] << ", "
						<< std::hex << (int)recv_data[8] << ", " << (int)recv_data[9] << ", " << (int)recv_data[10] << ", " << (int)recv_data[11] << ","
	 					<< std::hex << (int)recv_data[12] << ", " << (int)recv_data[13] << ", " << (int)recv_data[14] /*<< ", " << (int)recv_data[15] << ", "
						<< std::hex << (int)recv_data[16] << ", " << (int)recv_data[17] << ", " << (int)recv_data[18] << ", " << (int)recv_data[19] << ", "
						<< std::hex << (int)recv_data[20]*/<< std::endl;
			return true;
		}
	}
	
	
		
	return false;

	
}

int Read( unsigned char* buffer, int sz) {
	int size = 0;
	try {
		size = serial.Read(buffer, sz);
	}
	catch (std::exception &ex) {
		std::cout << "Read data from robot. CAUGHT EXCEPTION: " << std::endl;
	}


	return size;
}

int Write(unsigned char* data, int len) {
	int size = 0;
	size = serial.Write(data, len);


	return size;
}
void ReadInitsensors() {
	unsigned char data[11] = {0xee, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbb};
	Write(data, sizeof(data));
}
/****************************** Methods inherit DataReceivedNotify *********************/



int main(){
	
	serial = USBSerial();
	serial.Open("1111");
	
//	time_t timestamp;
//	timestamp =  getTimeStamp();
	
//	long int t = static_cast<long int>(timestamp);
//	std::cout << timestamp << ", " << t << std::endl;//毫秒级时间戳
	

	//std::thread th(RecFun);
	
	

	
	

while(true){

		ReadInitsensors();
		
		time_t timestamp;
		timestamp =  getTimeStamp();
		std::cout << timestamp << std::endl;
	
		long int t = static_cast<long int>(timestamp);
		
		if(t - timess > 40){
			//std::cout << "rec interput" << std::endl;
		}
		usleep(15000); // 10ms

	


	}

	
	return 0;
}


