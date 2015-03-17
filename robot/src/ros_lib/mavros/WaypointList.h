#ifndef _ROS_mavros_WaypointList_h
#define _ROS_mavros_WaypointList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mavros/Waypoint.h"

namespace mavros
{

  class WaypointList : public ros::Msg
  {
    public:
      uint8_t waypoints_length;
      mavros::Waypoint st_waypoints;
      mavros::Waypoint * waypoints;

    WaypointList():
      waypoints_length(0), waypoints(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = waypoints_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < waypoints_length; i++){
      offset += this->waypoints[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t waypoints_lengthT = *(inbuffer + offset++);
      if(waypoints_lengthT > waypoints_length)
        this->waypoints = (mavros::Waypoint*)realloc(this->waypoints, waypoints_lengthT * sizeof(mavros::Waypoint));
      offset += 3;
      waypoints_length = waypoints_lengthT;
      for( uint8_t i = 0; i < waypoints_length; i++){
      offset += this->st_waypoints.deserialize(inbuffer + offset);
        memcpy( &(this->waypoints[i]), &(this->st_waypoints), sizeof(mavros::Waypoint));
      }
     return offset;
    }

    const char * getType(){ return "mavros/WaypointList"; };
    const char * getMD5(){ return "9e87303e70921b453f266db2c0ff55c3"; };

  };

}
#endif