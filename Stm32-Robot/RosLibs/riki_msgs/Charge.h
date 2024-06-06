#ifndef _ROS_riki_msgs_Charge_h
#define _ROS_riki_msgs_Charge_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_msgs
{

  class Charge : public ros::Msg
  {
    public:
		typedef uint8_t _charge_type;
		_charge_type charge_pos;
    Charge():
    charge_pos(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
     
      uint16_t i_obstacle ;
			
			i_obstacle = this->charge_pos;
      *(outbuffer + offset + 0) = (i_obstacle >> (8 * 0)) & 0xFF;
			offset += sizeof(this->charge_pos);

      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint16_t i_obstacle ;
			
			i_obstacle = 0;
      i_obstacle |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->charge_pos = i_obstacle;
      offset += sizeof(this->charge_pos);
			
      return offset;
    }

    const char * getType(){ return "riki_msgs/Charge"; };
    const char * getMD5(){ return "eb02df81b5311055aad74b1dc0e56014"; };

  };

}
#endif


