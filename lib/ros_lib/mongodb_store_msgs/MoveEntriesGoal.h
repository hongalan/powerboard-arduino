#ifndef _ROS_mongodb_store_msgs_MoveEntriesGoal_h
#define _ROS_mongodb_store_msgs_MoveEntriesGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mongodb_store_msgs/StringList.h"
#include "ros/duration.h"

namespace mongodb_store_msgs
{

  class MoveEntriesGoal : public ros::Msg
  {
    public:
      typedef const char* _database_type;
      _database_type database;
      typedef mongodb_store_msgs::StringList _collections_type;
      _collections_type collections;
      typedef ros::Duration _move_before_type;
      _move_before_type move_before;
      typedef bool _delete_after_move_type;
      _delete_after_move_type delete_after_move;

    MoveEntriesGoal():
      database(""),
      collections(),
      move_before(),
      delete_after_move(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_database = strlen(this->database);
      varToArr(outbuffer + offset, length_database);
      offset += 4;
      memcpy(outbuffer + offset, this->database, length_database);
      offset += length_database;
      offset += this->collections.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->move_before.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->move_before.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->move_before.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->move_before.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->move_before.sec);
      *(outbuffer + offset + 0) = (this->move_before.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->move_before.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->move_before.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->move_before.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->move_before.nsec);
      union {
        bool real;
        uint8_t base;
      } u_delete_after_move;
      u_delete_after_move.real = this->delete_after_move;
      *(outbuffer + offset + 0) = (u_delete_after_move.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->delete_after_move);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_database;
      arrToVar(length_database, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_database; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_database-1]=0;
      this->database = (char *)(inbuffer + offset-1);
      offset += length_database;
      offset += this->collections.deserialize(inbuffer + offset);
      this->move_before.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->move_before.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->move_before.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->move_before.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->move_before.sec);
      this->move_before.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->move_before.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->move_before.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->move_before.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->move_before.nsec);
      union {
        bool real;
        uint8_t base;
      } u_delete_after_move;
      u_delete_after_move.base = 0;
      u_delete_after_move.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->delete_after_move = u_delete_after_move.real;
      offset += sizeof(this->delete_after_move);
     return offset;
    }

    const char * getType(){ return "mongodb_store_msgs/MoveEntriesGoal"; };
    const char * getMD5(){ return "1ab5b43da52ad01f6c509148e0abdfae"; };

  };

}
#endif