#ifndef _ROS_rospeex_msgs_SignalProcessingHeader_h
#define _ROS_rospeex_msgs_SignalProcessingHeader_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rospeex_msgs
{

  class SignalProcessingHeader : public ros::Msg
  {
    public:
      typedef const char* _user_type;
      _user_type user;
      typedef const char* _request_id_type;
      _request_id_type request_id;

    SignalProcessingHeader():
      user(""),
      request_id("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_user = strlen(this->user);
      varToArr(outbuffer + offset, length_user);
      offset += 4;
      memcpy(outbuffer + offset, this->user, length_user);
      offset += length_user;
      uint32_t length_request_id = strlen(this->request_id);
      varToArr(outbuffer + offset, length_request_id);
      offset += 4;
      memcpy(outbuffer + offset, this->request_id, length_request_id);
      offset += length_request_id;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_user;
      arrToVar(length_user, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_user; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_user-1]=0;
      this->user = (char *)(inbuffer + offset-1);
      offset += length_user;
      uint32_t length_request_id;
      arrToVar(length_request_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_request_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_request_id-1]=0;
      this->request_id = (char *)(inbuffer + offset-1);
      offset += length_request_id;
     return offset;
    }

    const char * getType(){ return "rospeex_msgs/SignalProcessingHeader"; };
    const char * getMD5(){ return "0a6d1eb09f4bd483b527113d13391d0e"; };

  };

}
#endif