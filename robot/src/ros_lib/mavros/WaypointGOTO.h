#ifndef _ROS_SERVICE_WaypointGOTO_h
#define _ROS_SERVICE_WaypointGOTO_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mavros/Waypoint.h"

namespace mavros
{

static const char WAYPOINTGOTO[] = "mavros/WaypointGOTO";

  class WaypointGOTORequest : public ros::Msg
  {
    public:
      mavros::Waypoint waypoint;

    WaypointGOTORequest():
      waypoint()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->waypoint.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->waypoint.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return WAYPOINTGOTO; };
    const char * getMD5(){ return "aeb81a83418666b5948fe3f24d59594e"; };

  };

  class WaypointGOTOResponse : public ros::Msg
  {
    public:
      bool success;

    WaypointGOTOResponse():
      success(0)
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
     return offset;
    }

    const char * getType(){ return WAYPOINTGOTO; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class WaypointGOTO {
    public:
    typedef WaypointGOTORequest Request;
    typedef WaypointGOTOResponse Response;
  };

}
#endif
