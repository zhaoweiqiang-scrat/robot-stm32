#ifndef _ROS_riki_msgs_Angle_h
#define _ROS_riki_msgs_Angle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_msgs
{

  class Angle : public ros::Msg
  {
    public:
      typedef float _angle_type;
      _angle_type V_Angle;
      _angle_type H_Angle;

    Angle():
	  V_Angle(0),
	  H_Angle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_v_angle;
      u_v_angle.real = this->V_Angle;
      *(outbuffer + offset + 0) = (u_v_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_v_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_v_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->V_Angle);
	  union {
        float real;
        uint32_t base;
      } u_h_angle;
      u_h_angle.real = this->H_Angle;
      *(outbuffer + offset + 0) = (u_h_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_h_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_h_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_h_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->H_Angle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_v_angle;
      u_v_angle.base = 0;
      u_v_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_v_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_v_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->V_Angle = u_v_angle.real;
      offset += sizeof(this->V_Angle);
	  union {
        float real;
        uint32_t base;
      } u_h_angle;
      u_h_angle.base = 0;
      u_h_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_h_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_h_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_h_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->H_Angle = u_h_angle.real;
      offset += sizeof(this->H_Angle);
     return offset;
    }

    const char * getType(){ return "riki_msgs/Angle"; };
    const char * getMD5(){ return "4e5e44f2225ce2013549b9468cf104b8"; };

  };

}
#endif

