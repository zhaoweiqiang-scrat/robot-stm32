#ifndef _ROS_riki_msgs_Collision_h
#define _ROS_riki_msgs_Collision_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_msgs
{

  class Collision : public ros::Msg
  {
    public:
		typedef uint8_t _collision_type;
		_collision_type hardware_collision_front;
		_collision_type hardware_collision_later;
    Collision():
        hardware_collision_front(0),		
		hardware_collision_later(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
     
      uint16_t i_obstacle ;
			
			i_obstacle = this->hardware_collision_front;
      *(outbuffer + offset + 0) = (i_obstacle >> (8 * 0)) & 0xFF;
			offset += sizeof(this->hardware_collision_front);
			
			i_obstacle = this->hardware_collision_later;
      *(outbuffer + offset + 0) = (i_obstacle >> (8 * 0)) & 0xFF;
			offset += sizeof(this->hardware_collision_later);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint16_t i_obstacle ;
			
			i_obstacle = 0;
      i_obstacle |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->hardware_collision_front = i_obstacle;
      offset += sizeof(this->hardware_collision_front);
			
			i_obstacle = 0;
      i_obstacle |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->hardware_collision_later = i_obstacle;
      offset += sizeof(this->hardware_collision_later);
			
      return offset;
    }

    const char * getType(){ return "riki_msgs/Collision"; };
    const char * getMD5(){ return "53f0df42062c93e247ac4dc1ad8ad871"; };

  };

}
#endif



