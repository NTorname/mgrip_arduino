#ifndef _ROS_mgrip_arduino_pneu_gripper_h
#define _ROS_mgrip_arduino_pneu_gripper_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mgrip_arduino
{

  class pneu_gripper : public ros::Msg
  {
    public:
      typedef int8_t _grip_type;
      _grip_type grip;
      typedef int8_t _minPres_type;
      _minPres_type minPres;
      typedef int8_t _maxPres_type;
      _maxPres_type maxPres;

    pneu_gripper():
      grip(0),
      minPres(0),
      maxPres(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_grip;
      u_grip.real = this->grip;
      *(outbuffer + offset + 0) = (u_grip.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->grip);
      union {
        int8_t real;
        uint8_t base;
      } u_minPres;
      u_minPres.real = this->minPres;
      *(outbuffer + offset + 0) = (u_minPres.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->minPres);
      union {
        int8_t real;
        uint8_t base;
      } u_maxPres;
      u_maxPres.real = this->maxPres;
      *(outbuffer + offset + 0) = (u_maxPres.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->maxPres);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_grip;
      u_grip.base = 0;
      u_grip.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->grip = u_grip.real;
      offset += sizeof(this->grip);
      union {
        int8_t real;
        uint8_t base;
      } u_minPres;
      u_minPres.base = 0;
      u_minPres.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->minPres = u_minPres.real;
      offset += sizeof(this->minPres);
      union {
        int8_t real;
        uint8_t base;
      } u_maxPres;
      u_maxPres.base = 0;
      u_maxPres.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->maxPres = u_maxPres.real;
      offset += sizeof(this->maxPres);
     return offset;
    }

    const char * getType(){ return "mgrip_arduino/pneu_gripper"; };
    const char * getMD5(){ return "c9140d77a787c3e831a060be683b4b5c"; };

  };

}
#endif