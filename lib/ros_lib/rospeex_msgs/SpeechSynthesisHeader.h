#ifndef _ROS_rospeex_msgs_SpeechSynthesisHeader_h
#define _ROS_rospeex_msgs_SpeechSynthesisHeader_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rospeex_msgs
{

  class SpeechSynthesisHeader : public ros::Msg
  {
    public:
      typedef int8_t _request_type_type;
      _request_type_type request_type;
      typedef const char* _engine_type;
      _engine_type engine;
      typedef const char* _voice_font_type;
      _voice_font_type voice_font;
      typedef const char* _language_type;
      _language_type language;
      typedef const char* _user_type;
      _user_type user;
      typedef const char* _request_id_type;
      _request_id_type request_id;
      enum { REQUEST_TYPE_SAY = 1 };
      enum { REQUEST_TYPE_TTS = 2 };

    SpeechSynthesisHeader():
      request_type(0),
      engine(""),
      voice_font(""),
      language(""),
      user(""),
      request_id("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_request_type;
      u_request_type.real = this->request_type;
      *(outbuffer + offset + 0) = (u_request_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->request_type);
      uint32_t length_engine = strlen(this->engine);
      varToArr(outbuffer + offset, length_engine);
      offset += 4;
      memcpy(outbuffer + offset, this->engine, length_engine);
      offset += length_engine;
      uint32_t length_voice_font = strlen(this->voice_font);
      varToArr(outbuffer + offset, length_voice_font);
      offset += 4;
      memcpy(outbuffer + offset, this->voice_font, length_voice_font);
      offset += length_voice_font;
      uint32_t length_language = strlen(this->language);
      varToArr(outbuffer + offset, length_language);
      offset += 4;
      memcpy(outbuffer + offset, this->language, length_language);
      offset += length_language;
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
      union {
        int8_t real;
        uint8_t base;
      } u_request_type;
      u_request_type.base = 0;
      u_request_type.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->request_type = u_request_type.real;
      offset += sizeof(this->request_type);
      uint32_t length_engine;
      arrToVar(length_engine, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_engine; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_engine-1]=0;
      this->engine = (char *)(inbuffer + offset-1);
      offset += length_engine;
      uint32_t length_voice_font;
      arrToVar(length_voice_font, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_voice_font; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_voice_font-1]=0;
      this->voice_font = (char *)(inbuffer + offset-1);
      offset += length_voice_font;
      uint32_t length_language;
      arrToVar(length_language, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_language; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_language-1]=0;
      this->language = (char *)(inbuffer + offset-1);
      offset += length_language;
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

    const char * getType(){ return "rospeex_msgs/SpeechSynthesisHeader"; };
    const char * getMD5(){ return "ebdb0a3273c98c277627de3b707a15ce"; };

  };

}
#endif