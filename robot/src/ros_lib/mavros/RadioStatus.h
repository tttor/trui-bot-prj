#ifndef _ROS_mavros_RadioStatus_h
#define _ROS_mavros_RadioStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mavros
{

  class RadioStatus : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t rssi;
      uint8_t remrssi;
      uint8_t txbuf;
      uint8_t noise;
      uint8_t remnoise;
      uint16_t rxerrors;
      uint16_t fixed;

    RadioStatus():
      header(),
      rssi(0),
      remrssi(0),
      txbuf(0),
      noise(0),
      remnoise(0),
      rxerrors(0),
      fixed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->rssi >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rssi);
      *(outbuffer + offset + 0) = (this->remrssi >> (8 * 0)) & 0xFF;
      offset += sizeof(this->remrssi);
      *(outbuffer + offset + 0) = (this->txbuf >> (8 * 0)) & 0xFF;
      offset += sizeof(this->txbuf);
      *(outbuffer + offset + 0) = (this->noise >> (8 * 0)) & 0xFF;
      offset += sizeof(this->noise);
      *(outbuffer + offset + 0) = (this->remnoise >> (8 * 0)) & 0xFF;
      offset += sizeof(this->remnoise);
      *(outbuffer + offset + 0) = (this->rxerrors >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rxerrors >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rxerrors);
      *(outbuffer + offset + 0) = (this->fixed >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fixed >> (8 * 1)) & 0xFF;
      offset += sizeof(this->fixed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->rssi =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->rssi);
      this->remrssi =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->remrssi);
      this->txbuf =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->txbuf);
      this->noise =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->noise);
      this->remnoise =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->remnoise);
      this->rxerrors =  ((uint16_t) (*(inbuffer + offset)));
      this->rxerrors |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->rxerrors);
      this->fixed =  ((uint16_t) (*(inbuffer + offset)));
      this->fixed |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->fixed);
     return offset;
    }

    const char * getType(){ return "mavros/RadioStatus"; };
    const char * getMD5(){ return "6f756e25cbec426b98a822e4ecb53638"; };

  };

}
#endif