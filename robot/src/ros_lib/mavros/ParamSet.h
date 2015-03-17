#ifndef _ROS_SERVICE_ParamSet_h
#define _ROS_SERVICE_ParamSet_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mavros
{

static const char PARAMSET[] = "mavros/ParamSet";

  class ParamSetRequest : public ros::Msg
  {
    public:
      const char* param_id;
      int64_t integer;
      float real;

    ParamSetRequest():
      param_id(""),
      integer(0),
      real(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_param_id = strlen(this->param_id);
      memcpy(outbuffer + offset, &length_param_id, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->param_id, length_param_id);
      offset += length_param_id;
      union {
        int64_t real;
        uint64_t base;
      } u_integer;
      u_integer.real = this->integer;
      *(outbuffer + offset + 0) = (u_integer.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_integer.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_integer.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_integer.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_integer.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_integer.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_integer.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_integer.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->integer);
      offset += serializeAvrFloat64(outbuffer + offset, this->real);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_param_id;
      memcpy(&length_param_id, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_param_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_param_id-1]=0;
      this->param_id = (char *)(inbuffer + offset-1);
      offset += length_param_id;
      union {
        int64_t real;
        uint64_t base;
      } u_integer;
      u_integer.base = 0;
      u_integer.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_integer.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_integer.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_integer.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_integer.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_integer.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_integer.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_integer.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->integer = u_integer.real;
      offset += sizeof(this->integer);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->real));
     return offset;
    }

    const char * getType(){ return PARAMSET; };
    const char * getMD5(){ return "ea705c97d21e1fbcbf5e474d576723e3"; };

  };

  class ParamSetResponse : public ros::Msg
  {
    public:
      bool success;
      int64_t integer;
      float real;

    ParamSetResponse():
      success(0),
      integer(0),
      real(0)
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
      union {
        int64_t real;
        uint64_t base;
      } u_integer;
      u_integer.real = this->integer;
      *(outbuffer + offset + 0) = (u_integer.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_integer.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_integer.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_integer.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_integer.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_integer.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_integer.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_integer.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->integer);
      offset += serializeAvrFloat64(outbuffer + offset, this->real);
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
      union {
        int64_t real;
        uint64_t base;
      } u_integer;
      u_integer.base = 0;
      u_integer.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_integer.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_integer.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_integer.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_integer.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_integer.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_integer.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_integer.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->integer = u_integer.real;
      offset += sizeof(this->integer);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->real));
     return offset;
    }

    const char * getType(){ return PARAMSET; };
    const char * getMD5(){ return "033326784a68a941a49106c3d258742e"; };

  };

  class ParamSet {
    public:
    typedef ParamSetRequest Request;
    typedef ParamSetResponse Response;
  };

}
#endif
