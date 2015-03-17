#ifndef _ROS_pr2_mechanism_msgs_JointStatistics_h
#define _ROS_pr2_mechanism_msgs_JointStatistics_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace pr2_mechanism_msgs
{

  class JointStatistics : public ros::Msg
  {
    public:
      const char* name;
      ros::Time timestamp;
      float position;
      float velocity;
      float measured_effort;
      float commanded_effort;
      bool is_calibrated;
      bool violated_limits;
      float odometer;
      float min_position;
      float max_position;
      float max_abs_velocity;
      float max_abs_effort;

    JointStatistics():
      name(""),
      timestamp(),
      position(0),
      velocity(0),
      measured_effort(0),
      commanded_effort(0),
      is_calibrated(0),
      violated_limits(0),
      odometer(0),
      min_position(0),
      max_position(0),
      max_abs_velocity(0),
      max_abs_effort(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      *(outbuffer + offset + 0) = (this->timestamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.sec);
      *(outbuffer + offset + 0) = (this->timestamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.nsec);
      offset += serializeAvrFloat64(outbuffer + offset, this->position);
      offset += serializeAvrFloat64(outbuffer + offset, this->velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->measured_effort);
      offset += serializeAvrFloat64(outbuffer + offset, this->commanded_effort);
      union {
        bool real;
        uint8_t base;
      } u_is_calibrated;
      u_is_calibrated.real = this->is_calibrated;
      *(outbuffer + offset + 0) = (u_is_calibrated.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_calibrated);
      union {
        bool real;
        uint8_t base;
      } u_violated_limits;
      u_violated_limits.real = this->violated_limits;
      *(outbuffer + offset + 0) = (u_violated_limits.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->violated_limits);
      offset += serializeAvrFloat64(outbuffer + offset, this->odometer);
      offset += serializeAvrFloat64(outbuffer + offset, this->min_position);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_position);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_abs_velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_abs_effort);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      this->timestamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.sec);
      this->timestamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.nsec);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->position));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->measured_effort));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->commanded_effort));
      union {
        bool real;
        uint8_t base;
      } u_is_calibrated;
      u_is_calibrated.base = 0;
      u_is_calibrated.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_calibrated = u_is_calibrated.real;
      offset += sizeof(this->is_calibrated);
      union {
        bool real;
        uint8_t base;
      } u_violated_limits;
      u_violated_limits.base = 0;
      u_violated_limits.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->violated_limits = u_violated_limits.real;
      offset += sizeof(this->violated_limits);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->odometer));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->min_position));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_position));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_abs_velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_abs_effort));
     return offset;
    }

    const char * getType(){ return "pr2_mechanism_msgs/JointStatistics"; };
    const char * getMD5(){ return "90fdc8acbce5bc783d8b4aec49af6590"; };

  };

}
#endif