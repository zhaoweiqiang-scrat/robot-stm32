#ifndef _ROS_riki_msgs_Illuminance_h
#define _ROS_riki_msgs_Illuminance_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_msgs
{

  class Illuminance : public ros::Msg
  {
    public:
      typedef uint32_t _illuminance_type;
      _illuminance_type light_intensity;

      Illuminance():
	  light_intensity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint16_t i_light ;
	  
      i_light = this->light_intensity;
      *(outbuffer + offset + 0) = (i_light >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (i_light >> (8 * 1)) & 0xFF;
			*(outbuffer + offset + 2) = (i_light >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (i_light >> (8 * 3)) & 0xFF;
      offset += sizeof(this->light_intensity);
	  
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint16_t i_light ;
	  
      i_light = 0;
      i_light |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      i_light |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
			i_light |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 2);
      i_light |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 3);
      this->light_intensity = i_light;
      offset += sizeof(this->light_intensity);
	  
      return offset;
    }

    const char * getType(){ return "riki_msgs/Illuminance"; };
    const char * getMD5(){ return "ccb260992d65deb70e99f35e599e080b"; };

  };

}
#endif


