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
    _ultrasonic_type ultr_FL;
	 // _ultrasonic_type ultr_FM;
		_ultrasonic_type ultr_FR;
		_ultrasonic_type ultr_L;
		_ultrasonic_type ultr_R;
		_ultrasonic_type ultr_AL;
		_ultrasonic_type ultr_AR;
    Ultrasonic():
	  ultr_FL(0),
	  //ultr_FM(0),
		ultr_FR(0),
		ultr_L(0),
	  ultr_R(0),
		ultr_AL(0),
		ultr_AR(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
 
	  union {
        float real;
        uint32_t base;
      } u_FL;
	  
      u_FL.real = this->ultr_FL;
      *(outbuffer + offset + 0) = (u_FL.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_FL.base >> (8 * 1)) & 0xFF;
	  *(outbuffer + offset + 2) = (u_FL.base >> (8 * 2)) & 0xFF;
	  *(outbuffer + offset + 3) = (u_FL.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ultr_FL);

	  union {
        float real;
        uint32_t base;
      } u_FM;
	  
//	  u_FM.real = this->ultr_FM;
//      *(outbuffer + offset + 0) = (u_FM.base >> (8 * 0)) & 0xFF;
//      *(outbuffer + offset + 1) = (u_FM.base >> (8 * 1)) & 0xFF;
//	  *(outbuffer + offset + 2) = (u_FM.base >> (8 * 2)) & 0xFF;
//	  *(outbuffer + offset + 3) = (u_FM.base >> (8 * 3)) & 0xFF;
//      offset += sizeof(this->ultr_FM);
		
			union {
        float real;
        uint32_t base;
      } u_FR;
	  
	  u_FR.real = this->ultr_FR;
      *(outbuffer + offset + 0) = (u_FR.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_FR.base >> (8 * 1)) & 0xFF;
	  *(outbuffer + offset + 2) = (u_FR.base >> (8 * 2)) & 0xFF;
	  *(outbuffer + offset + 3) = (u_FR.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ultr_FR);
			
			union {
        float real;
        uint32_t base;
      } u_L;
	  
	  u_L.real = this->ultr_L;
      *(outbuffer + offset + 0) = (u_L.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_L.base >> (8 * 1)) & 0xFF;
	  *(outbuffer + offset + 2) = (u_L.base >> (8 * 2)) & 0xFF;
	  *(outbuffer + offset + 3) = (u_L.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ultr_L);
			
			union {
        float real;
        uint32_t base;
      } u_R;
	  
	  u_R.real = this->ultr_R;
      *(outbuffer + offset + 0) = (u_R.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_R.base >> (8 * 1)) & 0xFF;
	  *(outbuffer + offset + 2) = (u_R.base >> (8 * 2)) & 0xFF;
	  *(outbuffer + offset + 3) = (u_R.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ultr_R);
			
			union {
        float real;
        uint32_t base;
      } u_AL;
	  
	  u_AL.real = this->ultr_AL;
      *(outbuffer + offset + 0) = (u_AL.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_AL.base >> (8 * 1)) & 0xFF;
	  *(outbuffer + offset + 2) = (u_AL.base >> (8 * 2)) & 0xFF;
	  *(outbuffer + offset + 3) = (u_AL.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ultr_AL);
			
			union {
        float real;
        uint32_t base;
      } u_AR;
	  
	  u_AR.real = this->ultr_AR;
      *(outbuffer + offset + 0) = (u_AR.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_AR.base >> (8 * 1)) & 0xFF;
	  *(outbuffer + offset + 2) = (u_AR.base >> (8 * 2)) & 0xFF;
	  *(outbuffer + offset + 3) = (u_AR.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ultr_AR);
			
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
  
	   union {
        float real;
        uint32_t base;
      } u_FL;
	  
      u_FL.base = 0;
      u_FL.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_FL.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_FL.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_FL.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ultr_FL = u_FL.real;
      offset += sizeof(this->ultr_FL);

//	  union {
//        float real;
//        uint32_t base;
//      } u_FM;
//	  
//      u_FM.base = 0;
//      u_FM.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
//      u_FM.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
//      u_FM.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
//      u_FM.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
//      this->ultr_FM = u_FM.real;
//      offset += sizeof(this->ultr_FM);
//			
//      return offset;
    }

    const char * getType(){ return "riki_msgs/Ultrasonic"; };
    const char * getMD5(){ return "e2db46b4b94104212c0b7f260ce00a1d"; };

  };

}
#endif


