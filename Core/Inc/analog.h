#ifndef ANALOG_H
#define ANALOG_H

#include <stdbool.h>

enum{Rload_10R = 10, Rload_5R = 5, Rload_2400R = 2400};

enum{GAIN_0 = 0, GAIN_1 = 1, GAIN_2 = 2, GAIN_5 = 5, GAIN_10 = 10,
     GAIN_20 = 20, GAIN_50 = 50, GAIN_100 = 100};

uint16_t GetU_HS();
uint16_t U_MR();
uint16_t GetR_ISO();
void SetSignalGain(uint8_t gain);
void ConnectRload(uint32_t Rload);
uint16_t GetB_HS();

void EnableCoilPower();
void DisableCoilPower();
void SetCoilPWMFreq(uint16_t freq); /*in Hz*/
uint32_t GetCoilPWMFreq(); /*in Hz*/
void SetCoilCurrent(uint16_t i); /*in mkA*/
void SetTestVolatge(uint16_t v); /*in mV*/
uint16_t MeashureCoilCurrent(); /*in mkA*/
void SetRSCurrent(uint32_t curr); /*in mA*/
uint32_t GetRSCurrent();
uint32_t GetCoilCurrent(); /*in mkA*/
void EnableCoilDrivePulses();
void DisableCoilDrivePulses();
uint16_t GetRS_ON();
uint16_t GetRS_OFF();/*in Omh*/
uint16_t GetURSon();/*in mV*/
uint16_t GetURSoff();/*in mV*/
uint32_t GetGoodCnt();
double GetDispertion();
void SetResistance_corrections(float offset, float gain);
uint32_t GetCommonCnt();
void AnalogMeashure();
void ClearDSPResult();
uint32_t GetTransientTime();
void SetDacValue(uint32_t rs_current);
uint16_t GetRS_ON_clear();

#endif