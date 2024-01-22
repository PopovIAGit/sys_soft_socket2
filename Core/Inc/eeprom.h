#ifndef EEPROM_H
#define EEPROM_H

#include "lwip.h"
#include <stdbool.h>

extern osMutexId memmutex_id;

typedef struct{
 struct{
   ip4_addr_t dev_ip;
   ip4_addr_t gw_ip;     
   ip4_addr_t netmask;
   uint64_t mac;
  }eth;   
 struct{
   float R_offset_GAIN_1;
   float R_offset_GAIN_2;
   float R_offset_GAIN_5;
   float R_offset_GAIN_10;
   float R_offset_GAIN_20;
   float R_offset_GAIN_50;
   float R_offset_GAIN_100;   
   float R_gain_GAIN_1;
   float R_gain_GAIN_2;
   float R_gain_GAIN_5;
   float R_gain_GAIN_10;
   float R_gain_GAIN_20;
   float R_gain_GAIN_50;
   float R_gain_GAIN_100; 
   float HS_offset;
   float R_ONE_GERKON;
   float R_TWO_GERKON;
   float R_THREE_GERKON;
 }calibrate;
 struct{
   uint16_t Icoil;
   uint16_t Fcoil;
   uint16_t Irs;
   uint16_t Type;
   uint16_t MaxRson;
   uint16_t MaxHS;
   uint16_t MaxDispersion;
  }testparam;
}devparam_t;

void CreateEEPROMSemaphore();
void ModuleSetParam(devparam_t *data);
void ModuleGetParam(devparam_t *data);

#endif