#ifndef _ROS_riki_msgs_Battery_h
#define _ROS_riki_msgs_Battery_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_msgs
{

  class Battery : public ros::Msg
  {
    public:
      typedef float _voltage_type;
      _voltage_type voltage;
      typedef float _current_type;
		  _current_type current;
		  typedef float _capacity_type;
		  _capacity_type surplus_capacity;
		  _capacity_type total_capacity;
		
		  typedef float _percent_type;
		  _percent_type surplus_capacity_percent;
		
		  typedef uint16_t _protection_type;
		  _protection_type state_protection;

    Battery():
      voltage(0),
		  current(0),
		  surplus_capacity(0),
		  total_capacity(0),
		  surplus_capacity_percent(0),
			state_protection(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.real = this->voltage;
      *(outbuffer + offset + 0) = (u_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage);
			union {
        float real;
        uint32_t base;
      } u_current;
      u_current.real = this->current;
      *(outbuffer + offset + 0) = (u_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current);
			
			union {
        float real;
        uint32_t base;
      } u_capacity;
			u_capacity.real = this->surplus_capacity;
      *(outbuffer + offset + 0) = (u_capacity.base >> (8 * 0)) & 0xFF;
			*(outbuffer + offset + 1) = (u_capacity.base >> (8 * 1)) & 0xFF;
			*(outbuffer + offset + 2) = (u_capacity.base >> (8 * 2)) & 0xFF;
			*(outbuffer + offset + 3) = (u_capacity.base >> (8 * 3)) & 0xFF;
			offset += sizeof(this->surplus_capacity);
			
			u_capacity.real = this->total_capacity;
      *(outbuffer + offset + 0) = (u_capacity.base >> (8 * 0)) & 0xFF;
			*(outbuffer + offset + 1) = (u_capacity.base >> (8 * 1)) & 0xFF;
			*(outbuffer + offset + 2) = (u_capacity.base >> (8 * 2)) & 0xFF;
			*(outbuffer + offset + 3) = (u_capacity.base >> (8 * 3)) & 0xFF;
			offset += sizeof(this->total_capacity);
			
			u_capacity.real = this->surplus_capacity_percent;
      *(outbuffer + offset + 0) = (u_capacity.base >> (8 * 0)) & 0xFF;
			*(outbuffer + offset + 1) = (u_capacity.base >> (8 * 1)) & 0xFF;
			*(outbuffer + offset + 2) = (u_capacity.base >> (8 * 2)) & 0xFF;
			*(outbuffer + offset + 3) = (u_capacity.base >> (8 * 3)) & 0xFF;
			offset += sizeof(this->surplus_capacity_percent);
			
			uint16_t i_protecton;
			i_protecton = this->state_protection;
			*(outbuffer + offset + 0) = (i_protecton >> (8 * 0)) & 0xFF;
			*(outbuffer + offset + 1) = (i_protecton >> (8 * 1)) & 0xFF;
			offset += sizeof(this->state_protection);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.base = 0;
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage = u_voltage.real;
      offset += sizeof(this->voltage);
			union {
        float real;
        uint32_t base;
      } u_current;
      u_current.base = 0;
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current = u_current.real;
      offset += sizeof(this->current);
			
			union {
        float real;
        uint32_t base;
      } u_capacity;
			
			u_capacity.base = 0;
      u_capacity.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
			u_capacity.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
			u_capacity.base |= ((uint16_t) (*(inbuffer + offset + 2))) << (8 * 2);
			u_capacity.base |= ((uint16_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->surplus_capacity = u_capacity.real;
      offset += sizeof(this->surplus_capacity);
			
			u_capacity.base = 0;
      u_capacity.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
			u_capacity.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
			u_capacity.base |= ((uint16_t) (*(inbuffer + offset + 2))) << (8 * 2);
			u_capacity.base |= ((uint16_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->total_capacity = u_capacity.real;
      offset += sizeof(this->total_capacity);;
			
			u_capacity.base = 0;
      u_capacity.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
			u_capacity.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
			u_capacity.base |= ((uint16_t) (*(inbuffer + offset + 2))) << (8 * 2);
			u_capacity.base |= ((uint16_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->surplus_capacity_percent = u_capacity.real;
      offset += sizeof(this->surplus_capacity_percent);

			uint16_t i_protection;
			i_protection = 0;
      i_protection |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
			i_protection |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->state_protection = i_protection;
      offset += sizeof(this->state_protection);
      return offset;
    }

    const char * getType(){ return "riki_msgs/Battery"; };
    const char * getMD5(){ return "18aff4c1bf7c34ec836b3c2289d8c855"; };

  };

}
#endif