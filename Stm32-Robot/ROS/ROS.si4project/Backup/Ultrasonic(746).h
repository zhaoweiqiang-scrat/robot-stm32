#ifndef _ROS_riki_msgs_Ultrasonic_h
#define _ROS_riki_msgs_Ultrasonic_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_msgs
{

  class Ultrasonic : public ros::Msg
  {
    public:
    typedef float _ultrasonic_type;
    _ultrasonic_type ultr_left;
	  _ultrasonic_type ultr_right;
    Ultrasonic():
	  ultr_left(0),
	  ultr_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
 
	  union {
        float real;
        uint32_t base;
      } u_left;
	  
      u_left.real = this->ultr_left;
      *(outbuffer + offset + 0) = (u_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left.base >> (8 * 1)) & 0xFF;
	  *(outbuffer + offset + 2) = (u_left.base >> (8 * 2)) & 0xFF;
	  *(outbuffer + offset + 3) = (u_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ultr_left);

	  union {
        float real;
        uint32_t base;
      } u_right;
	  
	  u_right.real = this->ultr_right;
      *(outbuffer + offset + 0) = (u_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right.base >> (8 * 1)) & 0xFF;
	  *(outbuffer + offset + 2) = (u_right.base >> (8 * 2)) & 0xFF;
	  *(outbuffer + offset + 3) = (u_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ultr_right);
		
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
  
	   union {
        float real;
        uint32_t base;
      } u_left;
	  
      u_left.base = 0;
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ultr_left = u_left.real;
      offset += sizeof(this->ultr_left);

	  union {
        float real;
        uint32_t base;
      } u_right;
	  
      u_right.base = 0;
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ultr_right = u_right.real;
      offset += sizeof(this->ultr_right);
			
      return offset;
    }

    const char * getType(){ return "riki_msgs/Ultrasonic"; };
    const char * getMD5(){ return "b49756fb4d915343d49930a70e5dcb93"; };

  };

}
#endif


