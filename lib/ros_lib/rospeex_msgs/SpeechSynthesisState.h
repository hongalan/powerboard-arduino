#ifndef _ROS_rospeex_msgs_SpeechSynthesisState_h
#define _ROS_rospeex_msgs_SpeechSynthesisState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rospeex_msgs/SpeechSynthesisHeader.h"

namespace rospeex_msgs
{

  class SpeechSynthesisState : public ros::Msg
  {
    public:
      typedef rospeex_msgs::SpeechSynthesisHeader _header_type;
      _header_type header;
      typedef bool _play_state_type;
      _play_state_type play_state;

    SpeechSynthesisState():
      header(),
      play_state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_play_state;
      u_play_state.real = this->play_state;
      *(outbuffer + offset + 0) = (u_play_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->play_state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_play_state;
      u_play_state.base = 0;
      u_play_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->play_state = u_play_state.real;
      offset += sizeof(this->play_state);
     return offset;
    }

    const char * getType(){ return "rospeex_msgs/SpeechSynthesisState"; };
    const char * getMD5(){ return "02db54ae5da52e1e8889b49b90a27db1"; };

  };

}
#endif