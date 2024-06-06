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
    typedef uint16_t _ultrasonic_type;
    _ultrasonic_type ultr_distance1;
	  _ultrasonic_type ultr_distance2;
		typedef uint8_t _infrared_type;
		_infrared_type obstacle_left;
		_infrared_type obstacle_right;
    Ultrasonic():
	  ultr_distance1(0),
	  ultr_distance2(0),
    obstacle_left(0),		
		obstacle_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
     
      uint16_t i_distance ;
	  
      i_distance = this->ultr_distance1;
      *(outbuffer + offset + 0) = (i_distance >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (i_distance >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ultr_distance1);
	  
	    i_distance = this->ultr_distance2;
      *(outbuffer + offset + 0) = (i_distance >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (i_distance >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ultr_distance2);
			
			i_distance = this->obstacle_left;
      *(outbuffer + offset + 0) = (i_distance >> (8 * 0)) & 0xFF;
			offset += sizeof(this->obstacle_left);
			
			i_distance = this->obstacle_right;
      *(outbuffer + offset + 0) = (i_distance >> (8 * 0)) & 0xFF;
			offset += sizeof(this->obstacle_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint16_t i_distance ;
	  
      i_distance = 0;
      i_distance |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      i_distance |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ultr_distance1 = i_distance;
      offset += sizeof(this->ultr_distance1);
	  
	    i_distance = 0;
      i_distance |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      i_distance |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ultr_distance2 = i_distance;
      offset += sizeof(this->ultr_distance2);
			
			i_distance = 0;
      i_distance |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->obstacle_left = i_distance;
      offset += sizeof(this->obstacle_left);
			
			i_distance = 0;
      i_distance |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->obstacle_right = i_distance;
      offset += sizeof(this->obstacle_right);
			
      return offset;
    }

    const char * getType(){ return "riki_msgs/Ultrasonic"; };
    const char * getMD5(){ return "da7225c3562dd2b9b88bf15aa7eb6a9e"; };

  };

}
#endif


