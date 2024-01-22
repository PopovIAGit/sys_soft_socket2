#include "stdint.h"
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "string.h"
#include "stdbool.h"
#include "stm32f7xx_hal_adc.h"
#include <math.h>
#include "analog.h"
#include "main.h"
#include <task.h>
#include "process.h"
#include "eeprom.h"

extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;
extern SPI_HandleTypeDef hspi1;
extern DAC_HandleTypeDef hdac;
extern I2C_HandleTypeDef hi2c1;

extern uint8_t clearDispersionFlag;

extern Tdev_cntrl dev_cntrl;

typedef struct {                                 // Message object structure
  uint16_t R_RS[BUFSIZE / 4];
}T_MEAS;

osMailQDef(mail, 3, T_MEAS);                    // Define mail queue
osMailQId  mail;

#pragma location = "VARIABLE_MASS"
uint32_t adc_data_array[BUFSIZE]; /* dual mode adc 2&1  1@3*/
static uint16_t U_HS_avg, U_MR_avg, R_RSon_avg, U_CSoff_avg, U_RSon_avg, U_RSoff_avg;
static uint32_t SHUNT_R = 10;
static volatile uint32_t gain = GAIN_10;
static uint32_t rs_current, coil_current;

float RS_offset, RS_gain;

void ConnectRload(uint32_t Rload){
 HAL_GPIO_WritePin(conn_10R_GPIO_Port, conn_10R_Pin, GPIO_PIN_RESET);    
 HAL_GPIO_WritePin(conn_5R_GPIO_Port, conn_5R_Pin, GPIO_PIN_RESET);    
 HAL_GPIO_WritePin(conn_2400R_GPIO_Port, conn_2400R_Pin, GPIO_PIN_RESET);    
  
 if(Rload == Rload_10R){
   HAL_GPIO_WritePin(conn_10R_GPIO_Port, conn_10R_Pin, GPIO_PIN_SET);       
   SHUNT_R = 10;
  } 
 if(Rload == Rload_5R){
   HAL_GPIO_WritePin(conn_5R_GPIO_Port, conn_5R_Pin, GPIO_PIN_SET);       
   SHUNT_R = 4;
  } 
 if(Rload == Rload_2400R){
   HAL_GPIO_WritePin(conn_2400R_GPIO_Port, conn_2400R_Pin, GPIO_PIN_SET);       
   SHUNT_R = 2400;
  }  
}

static uint32_t TEST_VLT;
static uint16_t TEST_VLT1 = 0;
#define InRange(Val, Min, Max)	(((Val) >= (Min)) && ((Val) <= (Max)))

void SetRSCurrent(uint32_t curr){
    
 switch(curr){
    case 1 : {
      SetTestVolatge(2040);   
      TEST_VLT = 2040;
      ConnectRload(Rload_2400R);
      rs_current = 1;
      break;
     }
    case 50 : { 
      TEST_VLT1 = 450;
      SetTestVolatge(TEST_VLT1);
      ConnectRload(Rload_10R);
      rs_current = 50;
      break;
     }
    case 500 : { 
      TEST_VLT1 = 1700;
      SetTestVolatge(TEST_VLT1);
      ConnectRload(Rload_5R);
      rs_current = 500;
      break;
     }
   }
}

void SetDacValue(uint32_t rs_current)
{
  if (rs_current == 50)
  { 
      if(!InRange(U_CSoff_avg, 515, 520))  // U_CSoff_avg = 518
      {        
        if (U_CSoff_avg < 515)
        {
            if (TEST_VLT1++ > 800) TEST_VLT1 = 600;
            SetTestVolatge(TEST_VLT1);
            osDelay(10);
        }
        else if (U_CSoff_avg > 520)
        {
            if (TEST_VLT1-- < 400) TEST_VLT1 = 400;
            SetTestVolatge(TEST_VLT1);
            osDelay(10);
        }
      }
      TEST_VLT = TEST_VLT1;
  }
  else if (rs_current == 500)
  {
     
      if(!InRange(U_CSoff_avg, 1825, 1832))  // U_CSoff_avg = 1829
      {        
        if (U_CSoff_avg < 1825)
        {
            if (TEST_VLT1++ > 2400) TEST_VLT1 = 2400;
            SetTestVolatge(TEST_VLT1);
            osDelay(10);
        }
        else if (U_CSoff_avg > 1832)
        {
            if (TEST_VLT1-- < 1500) TEST_VLT1 = 1500;
            SetTestVolatge(TEST_VLT1);
            osDelay(10);
        }
      }
        TEST_VLT = TEST_VLT1;
  }
  else if (rs_current == 1)
    return;
}



uint32_t GetRSCurrent(){
 return  rs_current;
}

void EnableCoilPower(){
 HAL_GPIO_WritePin(EN_COILPWR_GPIO_Port, EN_COILPWR_Pin, GPIO_PIN_SET); 
}

void DisableCoilPower(){
 HAL_GPIO_WritePin(EN_COILPWR_GPIO_Port, EN_COILPWR_Pin, GPIO_PIN_RESET); 
}

void EnableCoilDrivePulses(){
 HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);  
}

void DisableCoilDrivePulses(){
 HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1); 
}

void SetCoilPWMFreq(uint16_t freq){ /*in Hz*/
 if(freq > 100) freq = 100; 
 htim1.Instance->ARR =  40000 / freq;
}

uint32_t GetCoilPWMFreq(){
 if(htim1.Instance->ARR != 0) return (40000 / htim1.Instance->ARR);
 return 0; 
}

void SetCoilCurrent(uint16_t i){ /*in mkA*/
 uint64_t u;
 uint16_t n;
 uint8_t byte;
 
 const uint16_t R = 2;

 u = R * i * 36; /*mkV*/
 n = u * 65535 / 3000000; 

 HAL_Delay(1); 
 HAL_GPIO_WritePin(SPI_nCS_GPIO_Port, SPI_nCS_Pin, GPIO_PIN_RESET); 
 HAL_Delay(1);
 
 byte = 0x00;
 HAL_SPI_Transmit(&hspi1, &byte, 1, 1000);
 byte = n >> 8;
 HAL_SPI_Transmit(&hspi1, &byte, 1, 1000);
 byte = n;
 HAL_SPI_Transmit(&hspi1, &byte, 1, 1000);

 HAL_Delay(1);
 HAL_GPIO_WritePin(SPI_nCS_GPIO_Port, SPI_nCS_Pin, GPIO_PIN_SET); 
 HAL_Delay(1);
 coil_current = i;
}

uint32_t GetCoilCurrent(){
 return coil_current; 
}

void SetTestVolatge(uint16_t v){ /*in mV*/
 uint32_t n;
 const uint32_t Uref = 2500;
 n = v * 4095 / Uref;   
 HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, n); 
}

static uint16_t swap_bytes(uint16_t data){
 uint16_t res;
 
 res = (data & 0x00ff) << 8;
 res |= (data & 0xff00) >> 8;  
 return res;
}

uint16_t MeashureCoilCurrent(){/*in mkA*/
 #define CONFIGURATION_REG  0x00
 #define SHUNT_VLT_REG  0x01 
 #define BUS_VLT_REG  0x02 
 
 int16_t data;
 const uint16_t R = 1;
 
 if(HAL_I2C_Mem_Read(&hi2c1, 0x80, CONFIGURATION_REG, 1, (uint8_t*)&data, 2, 1000) != HAL_OK)
   asm("NOP");
 data = swap_bytes(data);
 if(data != 0x4527){
   data = swap_bytes(0x4527); 
   HAL_I2C_Mem_Write(&hi2c1, 0x80, CONFIGURATION_REG, 1, (uint8_t*)&data, 2, 1000);
  }
 if(HAL_I2C_Mem_Read(&hi2c1, 0x80, SHUNT_VLT_REG, 1, (uint8_t*)&data, 2, 1000) != HAL_OK)
   asm("NOP");
 data = abs(swap_bytes(data));

 data = data * 5 / 2; /*2.5*/ /*in mkv*/
 data = data / R; /*in mkv*/
 return data;
}

void SetSignalGain(uint8_t setgain){
  switch(setgain){
     case GAIN_0 : {
       HAL_GPIO_WritePin(GAIN_0_GPIO_Port, GAIN_0_Pin, GPIO_PIN_RESET); 
       HAL_GPIO_WritePin(GAIN_1_GPIO_Port, GAIN_1_Pin, GPIO_PIN_RESET); 
       HAL_GPIO_WritePin(GAIN_2_GPIO_Port, GAIN_2_Pin, GPIO_PIN_RESET);        
       gain = GAIN_0;
       break;
      }
     case GAIN_1 : {
       HAL_GPIO_WritePin(GAIN_0_GPIO_Port, GAIN_0_Pin, GPIO_PIN_SET); 
       HAL_GPIO_WritePin(GAIN_1_GPIO_Port, GAIN_1_Pin, GPIO_PIN_RESET); 
       HAL_GPIO_WritePin(GAIN_2_GPIO_Port, GAIN_2_Pin, GPIO_PIN_RESET);        
       gain = GAIN_1;
       break;
      }
     case GAIN_2 : {
       HAL_GPIO_WritePin(GAIN_0_GPIO_Port, GAIN_0_Pin, GPIO_PIN_RESET); 
       HAL_GPIO_WritePin(GAIN_1_GPIO_Port, GAIN_1_Pin, GPIO_PIN_SET); 
       HAL_GPIO_WritePin(GAIN_2_GPIO_Port, GAIN_2_Pin, GPIO_PIN_RESET);        
       gain = GAIN_2;
       break;
      }
     case GAIN_5 : {
       HAL_GPIO_WritePin(GAIN_0_GPIO_Port, GAIN_0_Pin, GPIO_PIN_SET); 
       HAL_GPIO_WritePin(GAIN_1_GPIO_Port, GAIN_1_Pin, GPIO_PIN_SET); 
       HAL_GPIO_WritePin(GAIN_2_GPIO_Port, GAIN_2_Pin, GPIO_PIN_RESET);        
       gain = GAIN_5;
       break;
      }
     case GAIN_10 : {
       HAL_GPIO_WritePin(GAIN_0_GPIO_Port, GAIN_0_Pin, GPIO_PIN_RESET); 
       HAL_GPIO_WritePin(GAIN_1_GPIO_Port, GAIN_1_Pin, GPIO_PIN_RESET); 
       HAL_GPIO_WritePin(GAIN_2_GPIO_Port, GAIN_2_Pin, GPIO_PIN_SET);        
       gain = GAIN_10;
       break;
      }
     case GAIN_20 : {
       HAL_GPIO_WritePin(GAIN_0_GPIO_Port, GAIN_0_Pin, GPIO_PIN_SET); 
       HAL_GPIO_WritePin(GAIN_1_GPIO_Port, GAIN_1_Pin, GPIO_PIN_RESET); 
       HAL_GPIO_WritePin(GAIN_2_GPIO_Port, GAIN_2_Pin, GPIO_PIN_SET);        
       gain = GAIN_20;
       break;
      }
     case GAIN_50 : {
       HAL_GPIO_WritePin(GAIN_0_GPIO_Port, GAIN_0_Pin, GPIO_PIN_RESET); 
       HAL_GPIO_WritePin(GAIN_1_GPIO_Port, GAIN_1_Pin, GPIO_PIN_SET); 
       HAL_GPIO_WritePin(GAIN_2_GPIO_Port, GAIN_2_Pin, GPIO_PIN_SET);        
       gain = GAIN_50;
       break;
      }
     case GAIN_100 : {
       HAL_GPIO_WritePin(GAIN_0_GPIO_Port, GAIN_0_Pin, GPIO_PIN_SET); 
       HAL_GPIO_WritePin(GAIN_1_GPIO_Port, GAIN_1_Pin, GPIO_PIN_SET); 
       HAL_GPIO_WritePin(GAIN_2_GPIO_Port, GAIN_2_Pin, GPIO_PIN_SET);        
       gain = GAIN_100;
       break;
      }
    }
}

/*{ADC2, ACD1}*/

void TIM1_UP_TIM10_IRQHandler(void){
 static uint32_t U_HS_summa_shadow, U_MR_summa_shadow, idx;
 static uint32_t U_CS_summa_shadow = 0, U_RS_summa_shadow = 0;
 static uint32_t I, U, idy;
 uint32_t *ptr;
 const uint32_t UREF = 2500;
 static bool polarity = false;
 T_MEAS  *mptr;
 #pragma location = "VARIABLE_MASS"
 static uint16_t R_RS[BUFSIZE / 4];
  
 if(!polarity){
   polarity = true;
   HAL_GPIO_WritePin(PWM_GPIO_Port, PWM_Pin, GPIO_PIN_SET);
   HAL_ADCEx_MultiModeStop_DMA(&hadc1);
   HAL_ADC_Start(&hadc2);
   HAL_DMA_Start(&hdma_adc1, (uint32_t)(ADC1_BASE + 0x308), (uint32_t)&adc_data_array[0], BUFSIZE / 2); 
   HAL_ADCEx_MultiModeStart_DMA(&hadc1,(uint32_t*)&adc_data_array[0] , BUFSIZE / 2);
     
   //calculate data during reed switch was off   
   U_HS_summa_shadow = 0; U_MR_summa_shadow = 0; idy = 0;
   U_RS_summa_shadow = 0; U_CS_summa_shadow = 0; 
   ptr = &adc_data_array[BUFSIZE / 2];   
    for(idx = 0; idx < BUFSIZE / 2; idx = idx + 4)
   {
     U_HS_summa_shadow += ptr[idx] >> 16;
     U_HS_summa_shadow += ptr[idx + 2] >> 16;
     U_MR_summa_shadow += ptr[idx + 1] >> 16;
     U_MR_summa_shadow += ptr[idx + 3] >> 16;     
     U_CS_summa_shadow += (ptr[idx + 1] & 0xffff) * UREF / 4096; //mV
     U_CS_summa_shadow += (ptr[idx + 3] & 0xffff) * UREF / 4096; //mV
    }
   
   U_HS_avg = U_HS_summa_shadow / (BUFSIZE / 4); //4 samples in one cycle
   U_MR_avg = U_MR_summa_shadow / (BUFSIZE / 4); //4 samples in one cycle
   U_CSoff_avg = U_CS_summa_shadow / (BUFSIZE / 4); //4 samples in one cycle
   U_RSoff_avg = U_RS_summa_shadow / (BUFSIZE / 4); //4 samples in one cycle   */
  }
 else{
   polarity = false;
   if(!IsContinuouslyMode()) HAL_GPIO_WritePin(PWM_GPIO_Port, PWM_Pin, GPIO_PIN_RESET);
   HAL_ADCEx_MultiModeStop_DMA(&hadc1);
   HAL_ADC_Start(&hadc2);
   HAL_DMA_Start(&hdma_adc1, (uint32_t)(ADC1_BASE + 0x308), (uint32_t)&adc_data_array[BUFSIZE / 2], BUFSIZE / 2);          
   HAL_ADCEx_MultiModeStart_DMA(&hadc1,(uint32_t*)&adc_data_array[BUFSIZE / 2] , BUFSIZE / 2);
  
   //calculate data during reed switch was on
   U_HS_summa_shadow = 0; U_MR_summa_shadow = 0; idy = 0;
   U_RS_summa_shadow = 0; 
   
   ptr = &adc_data_array[0];
   for(idx = 0; idx < BUFSIZE / 2; idx = idx + 4)
   {
     U_HS_summa_shadow += ptr[idx] >> 16;
     U_HS_summa_shadow += ptr[idx + 2] >> 16;
     U_MR_summa_shadow += ptr[idx + 1] >> 16;
     U_MR_summa_shadow += ptr[idx + 3] >> 16;
     
     I = (ptr[idx + 1] & 0xffff) * UREF / 4096 * 1000 / SHUNT_R; //mkA
     U = (ptr[idx] & 0xffff) * UREF / 4096 * 1000 / 694 * 100 / gain; //mkV
     U_RS_summa_shadow += (ptr[idx] & 0xffff) * UREF / 4096; //mV      
     if(I > 0) R_RS[idy++] = U * 1000 / I; //mOmh
     else R_RS[idy++] = 0xffff;
     I = (ptr[idx + 3] & 0xffff) * UREF / 4096 * 1000 / SHUNT_R; //mkA
     U = (ptr[idx + 2] & 0xffff) * UREF / 4096 * 1000 / 694 * 100 / gain ; //mkV
     U_RS_summa_shadow += (ptr[idx + 2] & 0xffff) * UREF / 4096; //mV       
     if(I > 0) R_RS[idy++] = U * 1000 / I; //mOmh
     else R_RS[idy++] = 0xffff;
    }

   U_HS_avg = U_HS_summa_shadow / (BUFSIZE / 4); //4 samples in one cycle
   U_MR_avg = U_MR_summa_shadow / (BUFSIZE / 4); //4 samples in one cycle
   U_RSon_avg = U_RS_summa_shadow / (BUFSIZE / 4); //4 samples in one cycle    
   mptr = osMailAlloc(mail, 0);
   if(mptr != NULL){
     memcpy((uint8_t*)mptr->R_RS, (uint8_t*)R_RS, sizeof(R_RS));
     osMailPut(mail, mptr);
    }
  }
 
 HAL_TIM_IRQHandler(&htim1); 
}



uint16_t GetU_HS(){/*in mV*/
  uint32_t U_HS;
  const uint32_t UREF = 3000;
  
  U_HS = (uint32_t)U_HS_avg * UREF / 4096 * 2;
  return (uint16_t)U_HS;
}

uint16_t GetB_HS(){ /*in mT*/
  uint16_t U_HS, B;
  const uint16_t K = 25; /*mV/mT*/
  const uint32_t UREF = 3000;
  
  U_HS = (uint32_t)U_HS_avg * UREF / 4096 * 2;/*in mV*/
  if(U_HS > 600) B = (U_HS - 600) / K;
  else B = 0;
  return (uint16_t)B;
}

uint16_t U_MR(){/*in mV*/
  uint32_t U_MR;
  const uint32_t UREF = 3000;
  
  U_MR = (uint32_t)U_MR_avg * UREF / 4096;
  return (uint16_t)U_MR;
}

uint16_t GetR_ISO(){/*in MOmh*/
  uint32_t U_MR, R_ISO;
  const uint32_t R_izm = 1; /*in MOmh*/
  const uint32_t UREF = 3000;
  
  U_MR = (uint32_t)U_MR_avg * UREF / 4096;    
  if((U_MR < TEST_VLT) && (U_MR !=0)) R_ISO = (TEST_VLT - U_MR) * R_izm *100/ U_MR ; //in MOmh
  else 
    R_ISO = 100;
  if(R_ISO > 32000) 
    R_ISO = 100;
  return (uint16_t)U_MR_avg;//R_ISO;
}

uint16_t GetRS_ON(){/*in mOmh*/
  float r = ((R_RSon_avg * RS_gain) + RS_offset);
  if (r > 0) return (uint16_t)r;
  else return 0;     
}

uint16_t GetRS_OFF(){/*in Omh*/
  uint32_t R_OFF;
  
  if((U_CSoff_avg < 520) && (U_CSoff_avg != 0)) 
    R_OFF = (520 - U_CSoff_avg) * SHUNT_R / U_CSoff_avg; 
  else R_OFF = 50;
  if(R_OFF > 50) R_OFF = 50;
  return (uint16_t)R_OFF;
}

uint16_t GetURSon(){/*in mV*/

  return U_RSon_avg;
}

uint16_t GetURSoff(){/*in mV*/

  return U_RSoff_avg;
}

static uint32_t transient_time;
bool clearflg;

struct{
  double sqr_summa;
  uint32_t cnt;
  double avg;
  double result;
  uint32_t good_cnt;
 }dispertion;

static uint16_t middle_of_3(uint16_t a, uint16_t b, uint16_t c){
   uint16_t middle;

   if ((a <= b) && (a <= c)){
     middle = (b <= c) ? b : c;
    }
   else{
     if ((b <= a) && (b <= c)){
       middle = (a <= c) ? a : c;
      }
     else{
       middle = (a <= b) ? a : b;
      }
   }
 return middle;
}
uint32_t e_time1=0;



void AnalogMeashure(){
  T_MEAS  *rptr;
  osEvent  evt;
  #pragma location = "VARIABLE_MASS"
  static uint16_t diff_RS[BUFSIZE / 4];
  uint32_t idx, idy, s_time, e_time, diffRS_avg, R_RS_summa_shadow;

  mail = osMailCreate(osMailQ(mail), NULL);      // create mail queue
  
  clearflg = false;
  dispertion.avg = 0;
  dispertion.cnt = 0;    
  dispertion.result = 0;
  dispertion.good_cnt = 0;  
  
  HAL_TIM_Base_Start_IT(&htim1);
  
  for(;;){
    if(clearflg){
       clearflg = false;
       dispertion.avg = 0;
       dispertion.cnt = 0;  
       dispertion.result = 0;
       dispertion.good_cnt = 0;
      }
     evt = osMailGet(mail, osWaitForever);        // wait for mail
     if(evt.status == osEventMail){
       
      
       if (dev_cntrl.suzType == 1)
       {
         SetDacValue(rs_current);
       }else {
         if (rs_current == 1)  {
            SetTestVolatge(2040);
            TEST_VLT = 2040;
         }
         else if (rs_current == 50){
            SetTestVolatge(550);
            TEST_VLT = 550;
         }
         else if (rs_current == 500){ 
            SetTestVolatge(2040);
            TEST_VLT = 2040;
         }
       }

       rptr = evt.value.p;
       /*level trigger*/
       for(idx = 0; idx < BUFSIZE / 4 ; idx++){
         if(rptr->R_RS[idx] > 10000) rptr->R_RS[idx] = 10000;
        }

       /*middle point filter*/
       for(idx = 0; idx < BUFSIZE / 4 ; idx++){
         if(idx == 0){
           diff_RS[0] = middle_of_3(rptr->R_RS[idx], rptr->R_RS[idx], rptr->R_RS[idx + 1]);
           continue;
          }
         if(idx == (BUFSIZE / 4 - 2)){
           diff_RS[BUFSIZE / 4 - 2] = middle_of_3(rptr->R_RS[BUFSIZE / 4 - 2], rptr->R_RS[BUFSIZE / 4 - 1], rptr->R_RS[BUFSIZE / 4 - 1]);
           continue;
          }         
         if(idx == (BUFSIZE / 4 - 1)){
           diff_RS[BUFSIZE / 4 - 1] = rptr->R_RS[BUFSIZE / 4];
           continue;
          }         
         diff_RS[idx] = middle_of_3(rptr->R_RS[idx], rptr->R_RS[idx + 1], rptr->R_RS[idx + 2]);
        }
       /*df/dt*/
       for(idx = 0; idx < BUFSIZE / 4; idx++){
         if(idx == 0) {diff_RS[0] = 0; continue;}
         if(idx == (BUFSIZE / 4 - 1)) {diff_RS[BUFSIZE / 4 - 1] = 0; continue;}
         diff_RS[idx] =abs(diff_RS[idx + 1] - diff_RS[idx]);
        }
       /*find start point*/
       for(idx = 0; idx < BUFSIZE / 4 - 50; idx++){
         diffRS_avg = 0;
         for(idy = 0; idy < 50; idy++) diffRS_avg = diffRS_avg + diff_RS[idx + idy];
         diffRS_avg = diffRS_avg / 50;
         if(diffRS_avg > 10){
           s_time = idx;
           break;
          }
        }
       /*find stop point*/
       for(idx = s_time; idx < BUFSIZE / 4 - 50; idx++){
         diffRS_avg = 0;
         for(idy = 0; idy < 50; idy++) diffRS_avg = diffRS_avg + diff_RS[idx + idy];
         diffRS_avg = diffRS_avg / 50;
         if(diffRS_avg < 10){
           e_time = idx;
           break;
          }
        }
       transient_time = (e_time - s_time) * 1000000 / 833333;
       /*calculate RS on resistance*/
       R_RS_summa_shadow = 0;
       for(idx = e_time; idx < BUFSIZE / 4; idx++) 
         R_RS_summa_shadow += rptr->R_RS[idx];
       if(BUFSIZE / 4 != e_time) 
         R_RSon_avg = R_RS_summa_shadow / ((BUFSIZE / 4) - e_time);
       else 
         R_RSon_avg = 10000;
       e_time1 = e_time;
       if(dev_cntrl.activate_rstest){
       dispertion.avg += R_RSon_avg;
       if(R_RSon_avg < dev_cntrl.max_Rs) dispertion.good_cnt++; 
       dispertion.cnt++;
       dev_cntrl.cnt1k++;
       dispertion.sqr_summa = pow(R_RSon_avg - dispertion.avg / dispertion.cnt, 2);
       
       dispertion.result = sqrt(dispertion.sqr_summa / dispertion.cnt);
       }
       else 
       {
       
       }
       osMailFree(mail, rptr);                    // free memory allocated for mail
      }      
     osThreadYield(); 
    }


}

void ClearDSPResult(){
 clearflg = true; 
}

double GetDispertion(){
 return dispertion.result;
}

uint32_t GetGoodCnt(){
 return  dispertion.good_cnt;
}

uint32_t GetCommonCnt(){
 return  dispertion.cnt;
}

void SetResistance_corrections(float offset, float gain){
 RS_offset = offset;
 RS_gain = gain;
}

uint32_t GetTransientTime(){
 return transient_time; 
}

uint16_t GetRS_ON_clear()
{
  return R_RSon_avg; 
}