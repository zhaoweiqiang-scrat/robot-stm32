#ifndef _ROS_riki_msgs_Action_h
#define _ROS_riki_msgs_Action_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_msgs
{

  class Action : public ros::Msg
  {
    public:
		typedef uint8_t _action_type;
		_action_type robot_action;
    Action():
        robot_action(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
     
      uint8_t i_action ;
			
			i_action = this->robot_action;
      *(outbuffer + offset + 0) = (i_action >> (8 * 0)) & 0xFF;
			offset += sizeof(this->robot_action);
			
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t i_action ;
			
			i_action = 0;
      i_action |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->robot_action = i_action;
      offset += sizeof(this->robot_action);
			
      return offset;
    }

    const char * getType(){ return "riki_msgs/Action"; };
    const char * getMD5(){ return "36dfbb391fe57cf347e95d000afd82f6"; };

  };

}
#endif



