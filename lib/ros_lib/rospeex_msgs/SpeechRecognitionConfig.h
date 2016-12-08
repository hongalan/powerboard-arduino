#ifndef _ROS_SERVICE_SpeechRecognitionConfig_h
#define _ROS_SERVICE_SpeechRecognitionConfig_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rospeex_msgs
{

static const char SPEECHRECOGNITIONCONFIG[] = "rospeex_msgs/SpeechRecognitionConfig";

  class SpeechRecognitionConfigRequest : public ros::Msg
  {
    public:
      typedef const char* _engine_type;
      _engine_type engine;
      typedef const char* _language_type;
      _language_type language;

    SpeechRecognitionConfigRequest():
      engine(""),
      language("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_engine = strlen(this->engine);
      varToArr(outbuffer + offset, length_engine);
      offset += 4;
      memcpy(outbuffer + offset, this->engine, length_engine);
      offset += length_engine;
      uint32_t length_language = strlen(this->language);
      varToArr(outbuffer + offset, length_language);
      offset += 4;
      memcpy(outbuffer + offset, this->language, length_language);
      offset += length_language;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_engine;
      arrToVar(length_engine, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_engine; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_engine-1]=0;
      this->engine = (char *)(inbuffer + offset-1);
      offset += length_engine;
      uint32_t length_language;
      arrToVar(length_language, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_language; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_language-1]=0;
      this->language = (char *)(inbuffer + offset-1);
      offset += length_language;
     return offset;
    }

    const char * getType(){ return SPEECHRECOGNITIONCONFIG; };
    const char * getMD5(){ return "917f81c82a3f1256fb139388ff9e634c"; };

  };

  class SpeechRecognitionConfigResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    SpeechRecognitionConfigResponse():
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    const char * getType(){ return SPEECHRECOGNITIONCONFIG; };
    const char * getMD5(){ return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class SpeechRecognitionConfig {
    public:
    typedef SpeechRecognitionConfigRequest Request;
    typedef SpeechRecognitionConfigResponse Response;
  };

}
#endif
