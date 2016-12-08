#ifndef _ROS_rospeex_msgs_SpeechRecognitionResponse_h
#define _ROS_rospeex_msgs_SpeechRecognitionResponse_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rospeex_msgs/SpeechRecognitionHeader.h"

namespace rospeex_msgs
{

  class SpeechRecognitionResponse : public ros::Msg
  {
    public:
      typedef rospeex_msgs::SpeechRecognitionHeader _header_type;
      _header_type header;
      typedef const char* _message_type;
      _message_type message;
      typedef const char* _memo_type;
      _memo_type memo;

    SpeechRecognitionResponse():
      header(),
      message(""),
      memo("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      uint32_t length_memo = strlen(this->memo);
      varToArr(outbuffer + offset, length_memo);
      offset += 4;
      memcpy(outbuffer + offset, this->memo, length_memo);
      offset += length_memo;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
      uint32_t length_memo;
      arrToVar(length_memo, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_memo; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_memo-1]=0;
      this->memo = (char *)(inbuffer + offset-1);
      offset += length_memo;
     return offset;
    }

    const char * getType(){ return "rospeex_msgs/SpeechRecognitionResponse"; };
    const char * getMD5(){ return "50ef59da490e8da471ea50b8b356c8e8"; };

  };

}
#endif