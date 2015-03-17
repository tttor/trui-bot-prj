#ifndef _ROS_SERVICE_CommandLong_h
#define _ROS_SERVICE_CommandLong_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mavros
{

static const char COMMANDLONG[] = "mavros/CommandLong";

  class CommandLongRequest : public ros::Msg
  {
    public:
      uint16_t command;
      uint8_t confirmation;
      float param1;
      float param2;
      float param3;
      float param4;
      float param5;
      float param6;
      float param7;
      enum { CMD_DO_SET_MODE =  176 };
      enum { CMD_DO_JUMP =  177 };
      enum { CMD_DO_CHANGE_SPEED =  178 };
      enum { CMD_DO_SET_HOME =  179 };
      enum { CMD_DO_SET_RELAY =  181 };
      enum { CMD_DO_REPEAT_RELAY =  182 };
      enum { CMD_DO_SET_SERVO =  183 };
      enum { CMD_DO_REPEAT_SERVO =  184 };
      enum { CMD_DO_CONTROL_VIDEO =  200 };
      enum { CMD_DO_SET_ROI =  201 };
      enum { CMD_DO_MOUNT_CONTROL =  205 };
      enum { CMD_DO_SET_CAM_TRIGG_DIST =  206 };
      enum { CMD_DO_FENCE_ENABLE =  207 };
      enum { CMD_DO_PARACHUTE =  208 };
      enum { CMD_DO_INVERTED_FLOGHT =  210 };
      enum { CMD_DO_MOUNT_CONTROL_QUAT =  220 };
      enum { CMD_PREFLIGHT_CALIBRATION =  241 };
      enum { CMD_MISSION_START =  300 };
      enum { CMD_COMPONENT_ARM_DISARM =  400 };
      enum { CMD_START_RX_PAIR =  500 };

    CommandLongRequest():
      command(0),
      confirmation(0),
      param1(0),
      param2(0),
      param3(0),
      param4(0),
      param5(0),
      param6(0),
      param7(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->command >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->command >> (8 * 1)) & 0xFF;
      offset += sizeof(this->command);
      *(outbuffer + offset + 0) = (this->confirmation >> (8 * 0)) & 0xFF;
      offset += sizeof(this->confirmation);
      union {
        float real;
        uint32_t base;
      } u_param1;
      u_param1.real = this->param1;
      *(outbuffer + offset + 0) = (u_param1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_param1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_param1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_param1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->param1);
      union {
        float real;
        uint32_t base;
      } u_param2;
      u_param2.real = this->param2;
      *(outbuffer + offset + 0) = (u_param2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_param2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_param2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_param2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->param2);
      union {
        float real;
        uint32_t base;
      } u_param3;
      u_param3.real = this->param3;
      *(outbuffer + offset + 0) = (u_param3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_param3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_param3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_param3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->param3);
      union {
        float real;
        uint32_t base;
      } u_param4;
      u_param4.real = this->param4;
      *(outbuffer + offset + 0) = (u_param4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_param4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_param4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_param4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->param4);
      union {
        float real;
        uint32_t base;
      } u_param5;
      u_param5.real = this->param5;
      *(outbuffer + offset + 0) = (u_param5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_param5.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_param5.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_param5.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->param5);
      union {
        float real;
        uint32_t base;
      } u_param6;
      u_param6.real = this->param6;
      *(outbuffer + offset + 0) = (u_param6.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_param6.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_param6.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_param6.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->param6);
      union {
        float real;
        uint32_t base;
      } u_param7;
      u_param7.real = this->param7;
      *(outbuffer + offset + 0) = (u_param7.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_param7.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_param7.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_param7.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->param7);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->command =  ((uint16_t) (*(inbuffer + offset)));
      this->command |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->command);
      this->confirmation =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->confirmation);
      union {
        float real;
        uint32_t base;
      } u_param1;
      u_param1.base = 0;
      u_param1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_param1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_param1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_param1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->param1 = u_param1.real;
      offset += sizeof(this->param1);
      union {
        float real;
        uint32_t base;
      } u_param2;
      u_param2.base = 0;
      u_param2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_param2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_param2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_param2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->param2 = u_param2.real;
      offset += sizeof(this->param2);
      union {
        float real;
        uint32_t base;
      } u_param3;
      u_param3.base = 0;
      u_param3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_param3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_param3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_param3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->param3 = u_param3.real;
      offset += sizeof(this->param3);
      union {
        float real;
        uint32_t base;
      } u_param4;
      u_param4.base = 0;
      u_param4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_param4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_param4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_param4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->param4 = u_param4.real;
      offset += sizeof(this->param4);
      union {
        float real;
        uint32_t base;
      } u_param5;
      u_param5.base = 0;
      u_param5.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_param5.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_param5.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_param5.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->param5 = u_param5.real;
      offset += sizeof(this->param5);
      union {
        float real;
        uint32_t base;
      } u_param6;
      u_param6.base = 0;
      u_param6.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_param6.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_param6.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_param6.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->param6 = u_param6.real;
      offset += sizeof(this->param6);
      union {
        float real;
        uint32_t base;
      } u_param7;
      u_param7.base = 0;
      u_param7.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_param7.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_param7.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_param7.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->param7 = u_param7.real;
      offset += sizeof(this->param7);
     return offset;
    }

    const char * getType(){ return COMMANDLONG; };
    const char * getMD5(){ return "5972d4d8225f9d160bb683f0834762b4"; };

  };

  class CommandLongResponse : public ros::Msg
  {
    public:
      bool success;
      uint8_t result;

    CommandLongResponse():
      success(0),
      result(0)
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
      *(outbuffer + offset + 0) = (this->result >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
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
      this->result =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->result);
     return offset;
    }

    const char * getType(){ return COMMANDLONG; };
    const char * getMD5(){ return "1cd894375e4e3d2861d2222772894fdb"; };

  };

  class CommandLong {
    public:
    typedef CommandLongRequest Request;
    typedef CommandLongResponse Response;
  };

}
#endif
