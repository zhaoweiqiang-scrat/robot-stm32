#ifndef _ROS_SERVICE_SetCmd_h
#define _ROS_SERVICE_SetCmd_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_srvs
{

static const char SETCMD[] = "riki_srvs/SetCmd";

  class SetCmdRequest : public ros::Msg
  {
    public:
      typedef uint8_t _data_type;
      _data_type data;

    SetCmdRequest():
      data(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
     
      *(outbuffer + offset + 0) = (this->data >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      
      this->data = ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      offset += sizeof(this->data);
     return offset;
    }

    const char * getType(){ return SETCMD; };
    const char * getMD5(){ return "7c8164229e7d2c17eb95e9231617fdee"; };

  };

  class SetCmdResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;

    SetCmdResponse():
      success(0),
      message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
     return offset;
    }

    const char * getType(){ return SETCMD; };
    const char * getMD5(){ return "937c9679a518e3a18d831e57125ea522"; };

  };

  class SetCmd {
    public:
    typedef SetCmdRequest Request;
    typedef SetCmdResponse Response;
  };

}
#endif
