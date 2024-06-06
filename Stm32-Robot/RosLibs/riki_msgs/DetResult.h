#ifndef _ROS_riki_msgs_DetResult_h
#define _ROS_riki_msgs_DetResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_msgs
{

  class DetResult : public ros::Msg
  {
    public:
      typedef float _D_Score_type;
	  _D_Score_type D_Score;

	  typedef uint8_t _B_Flag_type;
      _B_Flag_type B_Flag;

    DetResult():
      D_Score(0.0),
      B_Flag(false)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_D_Score;
      u_D_Score.real = this->D_Score;
      *(outbuffer + offset + 0) = (u_D_Score.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_D_Score.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_D_Score.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_D_Score.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->D_Score);
      
	  uint8_t i_B_Flag ;
	  
      i_B_Flag = this->B_Flag;
      *(outbuffer + offset + 0) = (i_B_Flag >> (8 * 0)) & 0xFF;
      
      offset += sizeof(this->B_Flag);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_D_Score;
      u_D_Score.base = 0;
      u_D_Score.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_D_Score.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_D_Score.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_D_Score.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->D_Score = u_D_Score.real;
      offset += sizeof(this->D_Score);
      uint8_t i_B_Flag ;
      i_B_Flag = 0;
      i_B_Flag |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      
      this->B_Flag = i_B_Flag;
      offset += sizeof(this->B_Flag);
     return offset;
    }

    const char * getType(){ return "riki_msgs/DetResult"; };
    const char * getMD5(){ return "472ffc80ed230d2db7fda15411079ec0"; };

  };

}
#endif