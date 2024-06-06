#ifndef _ROS_riki_msgs_Infrared_h
#define _ROS_riki_msgs_Infrared_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_msgs
{

  class Infrared : public ros::Msg
  {
    public:
		typedef uint8_t _infrared_type;
		_infrared_type obstacle_left;
		_infrared_type obstacle_right;
    Infrared():
    obstacle_left(0),		
		obstacle_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
     
      uint16_t i_obstacle ;
			
			i_obstacle = this->obstacle_left;
      *(outbuffer + offset + 0) = (i_obstacle >> (8 * 0)) & 0xFF;
			offset += sizeof(this->obstacle_left);
			
			i_obstacle = this->obstacle_right;
      *(outbuffer + offset + 0) = (i_obstacle >> (8 * 0)) & 0xFF;
			offset += sizeof(this->obstacle_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint16_t i_obstacle ;
			
			i_obstacle = 0;
      i_obstacle |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->obstacle_left = i_obstacle;
      offset += sizeof(this->obstacle_left);
			
			i_obstacle = 0;
      i_obstacle |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->obstacle_right = i_obstacle;
      offset += sizeof(this->obstacle_right);
			
      return offset;
    }

    const char * getType(){ return "riki_msgs/Infrared"; };
    const char * getMD5(){ return "53f0df42062c93e247ac4dc1ad8ad871"; };

  };

}
#endif



