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
		_collision_type hardware_collision_front_l;
		_collision_type hardware_collision_front_r;
		_collision_type hardware_collision_later_l;
		_collision_type hardware_collision_later_r;
    Collision():
        hardware_collision_front_l(0),		
		hardware_collision_front_r(0),
		hardware_collision_later_l(0),
		hardware_collision_later_r(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
     
      uint16_t i_obstacle ;
			
			i_obstacle = this->hardware_collision_front_l;
      *(outbuffer + offset + 0) = (i_obstacle >> (8 * 0)) & 0xFF;
			offset += sizeof(this->hardware_collision_front_l);
			
			i_obstacle = this->hardware_collision_front_r;
      *(outbuffer + offset + 0) = (i_obstacle >> (8 * 0)) & 0xFF;
			offset += sizeof(this->hardware_collision_front_r);
			
			i_obstacle = this->hardware_collision_later_l;
      *(outbuffer + offset + 0) = (i_obstacle >> (8 * 0)) & 0xFF;
			offset += sizeof(this->hardware_collision_later_l);
			
			i_obstacle = this->hardware_collision_later_r;
      *(outbuffer + offset + 0) = (i_obstacle >> (8 * 0)) & 0xFF;
			offset += sizeof(this->hardware_collision_later_r);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint16_t i_obstacle ;
			
			i_obstacle = 0;
      i_obstacle |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->hardware_collision_front_l = i_obstacle;
      offset += sizeof(this->hardware_collision_front_l);
			
			i_obstacle = 0;
      i_obstacle |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->hardware_collision_front_r = i_obstacle;
      offset += sizeof(this->hardware_collision_front_r);
			
			i_obstacle = 0;
      i_obstacle |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->hardware_collision_later_l = i_obstacle;
      offset += sizeof(this->hardware_collision_later_l);
			
			i_obstacle = 0;
      i_obstacle |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->hardware_collision_later_r = i_obstacle;
      offset += sizeof(this->hardware_collision_later_r);
      return offset;
    }

    const char * getType(){ return "riki_msgs/Collision"; };
    const char * getMD5(){ return "b3d9ad910c842c0875ac80a752ea79fc"; };

  };

}
#endif



