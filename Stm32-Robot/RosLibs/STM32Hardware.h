#pragma once
#include "hardwareserial.h"
#include "usart.h"
HardwareSerial Serial(SERIAL2);
#define SERIAL_CLASS  HardwareSerial

class STM32Hardware {
  public:
  	STM32Hardware(SERIAL_CLASS* io , long baud= 115200){
      iostream = io;
      baud_ = baud;
    }
    STM32Hardware()
    {
      iostream = &Serial;
      baud_ = 115200;
    }
    STM32Hardware(STM32Hardware& h){
	  this->iostream = iostream;
      this->baud_ = h.baud_;
    }

    void setBaud(long baud){
      this->baud_= baud;
    }

    int getBaud(){return baud_;}

    void init(){
      iostream->begin(baud_);
    }

    int read(){
      //if(iostream->available())
			if(DMA_RX2_FLAG)//���յ����ݱ�־λ
			{
	  	return iostream->read();
      }
			else
			{
	    return -1;
      }
    };

    void write(uint8_t* data, int length){
      for(int i=0; i<length; i++){
		  iostream->write(data[i]);
      }
    }

    unsigned long time(){return millis();}

  protected:
    SERIAL_CLASS* iostream;
    long baud_;
};
