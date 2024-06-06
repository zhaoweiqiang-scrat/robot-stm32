#ifndef _ROS_riki_msgs_LED_h
#define _ROS_riki_msgs_LED_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_msgs
{

  class LED : public ros::Msg
  {
    public:
		typedef float _led_type;
		_led_type led_level;
    LED():
    led_level(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
       int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_led;
      u_led.real = this->led_level;
      *(outbuffer + offset + 0) = (u_led.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_led.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_led.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_led.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->led_level);
			return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_led;
      u_led.base = 0;
      u_led.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_led.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_led.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_led.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->led_level = u_led.real;
      offset += sizeof(this->led_level);
			
      return offset;
    }

    const char * getType(){ return "riki_msgs/LED"; };
    const char * getMD5(){ return "31b4fc4e62ee0cbe1cef3ab56a7ff9bf"; };

  };

}
#endif



