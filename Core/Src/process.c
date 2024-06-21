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
#define InRange(Val, Min, Max)	(((Val) >= (Min)) && ((Val) <= (Max)))

extern TIM_HandleTypeDef htim1;
uint8_t clearDispersionFlag = 0;

enum{SIGNEMODE, CYCLEMODE};
enum{SUZ_TOMZEL = 0, SUZ_OTHER = 1};
 uint32_t saved_RSon_old = 0;
#define LinearInterpolation(x1, y1, x2, y2, x) \
(y1 +  ( (y2 - y1)*(x - x1) )/(x2 - x1) ) 

/*
// Точки 		                темпер.  АЦП.        сопр.    №
TDot dots[NUM_DOTS] = {			4000,    650,	    //  525	0 455
					5000,    654,         //  691	1 484
					6000,	 660,	    //  820	2 500
					7000,	 664,	    //  1039	3 615
					8000,	 670,	    //  1379	4 710
					9000,    674,	    //  1670	5 790
					10000,   678,	    //  1774	6 810
					11000,	 684,	    //  1039	3 615
					12000,	 688,	    //  1379	4 710
					13000,   694,	    //  1670	5 790
					14000,   698,	    //  1774	6 810
					15000,	 704,	    //  1039	3 615
					16000,	 710,	    //  1379	4 710
					17000,   716,	    //  1670	5 790
					18000,   720,	    //  1774	6 810
					19000,	 726,	    //  1039	3 615
					20000,	 732,	    //  1379	4 710
					21000,   736,	    //  1670	5 790
					22000,   742,	    //  1774	6 810
					23000,	 748,	    //  1039	3 615
					24000,	 754,	    //  1379	4 710
					25000,   758,	    //  1670	5 790
					26000,   764,	    //  1774	6 810
					27000,	 768,	    //  1039	3 615
					28000,	 772,	    //  1379	4 710
					29000,   778,	    //  1670	5 790
					30000,   782,	    //  1774	6 810
					31000,	 788,	    //  1039	3 615
					32000,	 792,	    //  1379	4 710
					33000,   798,	    //  1670	5 790
					34000,   802,	    //  1774	6 810
					35000,   808,	    //  1774	6 810
					36000,   814,	    //  1774	6 810
					37000,   818,	    //  1774	6 810
					38000,   824,	    //  1774	6 810
					39000,   830,	    //  1774	6 810
					40000,   836};	    //  1882	7 840
 
 */
 
 


	// Точки 		                темпер.  АЦП.        сопр.    №
TDot dots[NUM_DOTS] = {			4000,   790,	    //  525	0 455
					5000,   810,         //  691	1 484
					6000,	828,	    //  820	2 500
					7000,	848,	    //  1039	3 615
					8000,	864,	    //  1379	4 710
					9000,   884,	    //  1670	5 790
					10000,  902,	    //  1774	6 810
					11000,	922,	    //  1039	3 615
					12000,	940,	    //  1379	4 710
					13000,   958,	    //  1670	5 790
					14000,   978,	    //  1774	6 810
					15000,	 996,	    //  1039	3 615
					16000,	 1014,	    //  1379	4 710
					17000,   1034,	    //  1670	5 790
					18000,   1052,	    //  1774	6 810
					19000,	 1072,	    //  1039	3 615
					20000,	 1090,	    //  1379	4 710
					21000,   1108,	    //  1670	5 790
					22000,   1126,	    //  1774	6 810
					23000,	 1146,	    //  1039	3 615
					24000,	 1164,	    //  1379	4 710
					25000,   1184,	    //  1670	5 790
					26000,   1202,	    //  1774	6 810
					27000,	 1220,	    //  1039	3 615
					28000,	 1240,	    //  1379	4 710
					29000,   1258,	    //  1670	5 790
					30000,   1276,	    //  1774	6 810
					31000,	 1296,	    //  1039	3 615
					32000,	 1312,	    //  1379	4 710
					33000,   1332,	    //  1670	5 790
					34000,   1352,	    //  1774	6 810
					35000,   1370,	    //  1774	6 810
					36000,   1390,	    //  1774	6 810
					37000,   1406,	    //  1774	6 810
					38000,   1426,	    //  1774	6 810
					39000,   1444,	    //  1774	6 810
					40000,   1464};	    //  1882	7 840


/*

// Точки 		                темпер.  АЦП.        сопр.    №
TDot dots[NUM_DOTS] = {			4000,   750,	    //  525	0 455
					5000,   754,         //  691	1 484
					6000,	760,	    //  820	2 500
					7000,	764,	    //  1039	3 615
					8000,	770,	    //  1379	4 710
					9000,   774,	    //  1670	5 790
					10000,  778,	    //  1774	6 810
					11000,	784,	    //  1039	3 615
					12000,	788,	    //  1379	4 710
					13000,   794,	    //  1670	5 790
					14000,   798,	    //  1774	6 810
					15000,	 804,	    //  1039	3 615
					16000,	 810,	    //  1379	4 710
					17000,   816,	    //  1670	5 790
					18000,   820,	    //  1774	6 810
					19000,	 826,	    //  1039	3 615
					20000,	 832,	    //  1379	4 710
					21000,   836,	    //  1670	5 790
					22000,   842,	    //  1774	6 810
					23000,	 848,	    //  1039	3 615
					24000,	 854,	    //  1379	4 710
					25000,   858,	    //  1670	5 790
					26000,   864,	    //  1774	6 810
					27000,	 868,	    //  1039	3 615
					28000,	 872,	    //  1379	4 710
					29000,   878,	    //  1670	5 790
					30000,   882,	    //  1774	6 810
					31000,	 888,	    //  1039	3 615
					32000,	 892,	    //  1379	4 710
					33000,   898,	    //  1670	5 790
					34000,   902,	    //  1774	6 810
					35000,   908,	    //  1774	6 810
					36000,   914,	    //  1774	6 810
					37000,   918,	    //  1774	6 810
					38000,   924,	    //  1774	6 810
					39000,   930,	    //  1774	6 810
					40000,   936};	    //  1882	7 840
 

*/

Tdev_cntrl dev_cntrl;

static devparam_t confparam;

static float button_rs = 0.0, button_hs = 0.0;
static bool buttrs_state = false, butths_state = false, butths_long_state = false, buttrs_long_state = false;

#define GREEN_LED 1
#define RED_LED 2
#define ULTRA_LONG_LED 30000
#define LONG_LED 5000
#define SHORT_LED 500
#define TOGGLE_LED 1
#define NOTOGGLE_LED 0   
// калибровка герконового модуля
#define ONE_GERKON 4800
#define TWO_GERKON 3000
#define THREE_GERKON 2200

/*
#define ONE_GERKON_RISO_UP 730
#define ONE_GERKON_RISO_DOWN 600


#define TWO_GERKON_RISO_UP 790
#define TWO_GERKON_RISO_DOWN 740


#define THREE_GERKON_RISO 800*/

#define GERKON_OFFSET 10
//-----------------------------
uint32_t led_work_time = SHORT_LED;
uint32_t led_test_time = LONG_LED;
uint32_t led_gerkon_time = 0;

uint32_t timerLed1 = 0;//osKernelSysTick();
uint32_t timerLed2 = 0;//osKernelSysTick();

     uint8_t nullFlag = 0;

static uint8_t LedWork = GREEN_LED;
static uint8_t LedTest = 0;
static uint8_t LedGercon = 0;


static uint32_t minR = 500000;
static uint32_t minRprev = 500000;

static uint32_t minRiso = 0;
static uint32_t minRisoFinale = 0;

static uint32_t maxHs = 0;
static uint32_t maxHSfinale = 0;

/*-----------------------------------------------------------------------------
ToDo List
1 поиск положения - READY
2 проверка геркона 2/3 - READY
3 АЦП датчика холла - READY
4 Подстройка катушки под поплавок - READY
5 Компановка веба - Готово
6 Подключение javascript -ready
7 Управление - долгое нажатие, короткое нажатие, отпускание и тд - READY
-------------------------------------------------------------------------------
Работы по стенду
1. опредедение МДС срабатывания и МДС отпускания
2. Режим автотестирования - разные токи + МДС + дисперсия по нажатию
-----------------------------------------------------------------------------*/
uint16_t HSTMP = 0;
//--------------------------------------------------------
void TempObserverInit(TTempObserver *p)
{
  int16_t i = 0;
  
  for (i = 0; i < NUM_DOTS; i++)
    p->dots[i] = dots[i];
  p->maxResist = KTY83_MAX_RESIST;
}
//--------------------------------------------------------
void TempObserverUpdate(TTempObserver *p)
{
  static int16_t i=0;
  
  // Значение АЦП меньше минимально-допустимого или больше минимально-допустимого
  if (p->inputR <= KTY83_MIN_RESIST)		
  {
   // p->fault = true;
     p->outputT = 4000;
    return;
  }
  else if (p->inputR >= KTY83_MAX_RESIST )
  {
    //p->fault = true;					// это значит обрыв датчика температуры или его сбой
    p->outputT = 40000;
    return;		
  }
  else 									// Значение АЦП в пределах
    p->fault = false;					// - нет аварии 
  
  // Определяем, между какими значениями dots находится R_входное
  //while (! ((p->inputR >= p->dots[i].resist)&&(p->inputR < p->dots[i+1].resist)) )	// Для сопротивления (прямая зависимость)
  while (! ((p->inputR >= p->dots[i].adc)&&(p->inputR < p->dots[i+1].adc)) )	// Для АЦП (обратная зависимость)
  {
    if (p->inputR > p->dots[i].adc)
    {
      i++;	// Движемся по характеристике вверх и вниз
      //	if(i > 7) i = 7;
    }
    else
    {
      i--;
      //	if(i < -1) i = -1;
    }							// пока не окажемся между двумя точками
  }
  
  if (i > NUM_DOTS) i = NUM_DOTS;
  else if (i < 0) i = 0;
  
  if (p->inputR == p->dots[i].adc)			// Если четко попали на точку
    p->outputT = p->dots[i].temper;		// берем значение температуры этой точки
  else// Линейная интерполяция			   в противном случае интерполируем
    p->outputT = LinearInterpolation(p->dots[i].adc, p->dots[i].temper ,p->dots[i+1].adc ,p->dots[i+1].temper, p->inputR);
}

void Algorithm(){
  
  uint32_t btnRsTimer = 0; //osKernelSysTick();
  uint32_t btnHsTimer = 0; //osKernelSysTick();
  enum{WAIT, CHOOSING_GAIN, TEST, CONTINUOUSLY_ON, POPLOVOK_TEST} fsm = WAIT;
  const uint32_t UREF = 2500;
  
  
  SetSignalGain(GAIN_1);
  osDelay(100);
  dev_cntrl.activate_rstest = false;
  dev_cntrl.float_test = false;
  dev_cntrl.RISO_test = false;
  dev_cntrl.led_blink = false;
  dev_cntrl.calibrateR = false;
  dev_cntrl.continuously_on = false;
  dev_cntrl.stable_on = false;
  dev_cntrl.findMinR = false;
  dev_cntrl.findR = false;
  dev_cntrl.max_Rs = 300;
  dev_cntrl.max_Dispersion = 100;
  dev_cntrl.max_Hs = 1;
  dev_cntrl.max_Hs = 740;
  dev_cntrl.saved_Riso = 0;
  dev_cntrl.TempObserver.inputR = 0;
  dev_cntrl.connect_on = true;
  
  TempObserverInit(&dev_cntrl.TempObserver); 
  DisableCoilPower();
  ModuleGetParam(&confparam);   
  
  SetCoilCurrent(confparam.testparam.Icoil);
  SetRSCurrent(confparam.testparam.Irs);
  SetCoilPWMFreq(confparam.testparam.Fcoil); 
  SetSuzType(confparam.testparam.Type);
  SetMaxDispersion(confparam.testparam.MaxDispersion);
  SetMaxHs(confparam.testparam.MaxHS);
  SetMaxRs(confparam.testparam.MaxRson);
  
  //dev_cntrl.activate_rstest = true;
  //dev_cntrl.mode = CYCLEMODE;
  //dev_cntrl.continuously_on = true;
  //HAL_TIM_Base_Start_IT(&htim1);
  
  for(;;){
    
    // indication----------------------------------------------------------------
    //  левый(Работа)
    
    HSTMP =  GetCalibHS();
    
    
    if (LedWork == RED_LED) {
      if (abs(timerLed2 - osKernelSysTick()) > led_work_time) {
	timerLed2 = osKernelSysTick();
	HAL_GPIO_TogglePin(WRKLED_R_GPIO_Port, WRKLED_R_Pin);
      }
    } else if (LedWork == GREEN_LED) {
      if (abs(timerLed2 - osKernelSysTick()) > led_work_time) {
	timerLed2 = osKernelSysTick();
	HAL_GPIO_TogglePin(WRKLED_G_GPIO_Port, WRKLED_G_Pin);
      }
    } else {
      HAL_GPIO_WritePin(WRKLED_R_GPIO_Port, WRKLED_R_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(WRKLED_G_GPIO_Port, WRKLED_G_Pin, GPIO_PIN_RESET);
    }
    
    //средний (ТЕСТ)
    
    if (LedTest == GREEN_LED) {
      HAL_GPIO_WritePin(TESTLED_R_GPIO_Port, TESTLED_G_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(TESTLED_G_GPIO_Port, TESTLED_R_Pin, GPIO_PIN_RESET);
      if (abs(timerLed1 - osKernelSysTick()) > led_test_time) {
	timerLed1 = osKernelSysTick();
	LedTest = 0;
      }
    } else if (LedTest == RED_LED) {
      HAL_GPIO_WritePin(TESTLED_R_GPIO_Port, TESTLED_G_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(TESTLED_G_GPIO_Port, TESTLED_R_Pin, GPIO_PIN_SET);
      if (abs(timerLed1 - osKernelSysTick()) > led_test_time) {
	timerLed1 = osKernelSysTick();
	LedTest = 0;
      }
    } else {
      HAL_GPIO_WritePin(TESTLED_R_GPIO_Port, TESTLED_R_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(TESTLED_G_GPIO_Port, TESTLED_G_Pin, GPIO_PIN_RESET);
    }
    
    // правый (ГЕРКОН)
    
    if (dev_cntrl.activate_rstest || dev_cntrl.continuously_on)
      LedGercon = 1;
    else
      LedGercon = 0;
    
    if (LedGercon) {
      HAL_GPIO_WritePin(RSLED_GPIO_Port, RSLED_Pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(RSLED_GPIO_Port, RSLED_Pin, GPIO_PIN_RESET);
    }
    
    // control-----------------------------------------------------------------
    
    uint32_t msRs = osKernelSysTick();
    uint32_t msHs = osKernelSysTick();
    uint8_t keyrs_state = HAL_GPIO_ReadPin(FLOAT_TEST_GPIO_Port, FLOAT_TEST_Pin);
    uint8_t keyhs_state = HAL_GPIO_ReadPin(RS_TEST_GPIO_Port, RS_TEST_Pin);
    
    // кнопка геркон
    if (!keyrs_state && !buttrs_state && (msRs - btnRsTimer) > 10)
    {
      buttrs_state = 1;
      buttrs_long_state = 0;
      btnRsTimer = msRs;
    }
    else if (!keyrs_state && !buttrs_long_state &&(msRs - btnRsTimer) > 2000)
    {
      buttrs_long_state = 1;
      // дейтсвие при длительном нажатии     
    }
    else if (keyrs_state && buttrs_state) 
    { 
      buttrs_state = 0;
      btnRsTimer = msRs;    
      if(!buttrs_long_state)// действие при коротком нажатии
      {
	if (dev_cntrl.suzType == SUZ_OTHER)
	{   
	  SetStatus(st_FIND_GERKON_R);
	  dev_cntrl.activate_rstest = true;
	  dev_cntrl.findR =true;
	  dev_cntrl.mode = SIGNEMODE;
	}
	else {
	  SetStatus(st_FIND_GERKON_MODUL_RISO);
	  SetOnContinuously(); 
	  dev_cntrl.findR =true;
	}
      }
      else // длинное нажатие
      {
	if (dev_cntrl.suzType == SUZ_OTHER){
	  SetStatus(st_FIND_GERKON);
	  SetCycleTestMode();
	  StartRSTest();
	  //  SetOnContinuously();
	  dev_cntrl.findMinR = true;
	}
	else {
	  SetStatus(st_FIND_GERKON_MODUL);
	  SetOnContinuously();
	  dev_cntrl.findMinR = true;
	}
      }
      buttrs_long_state = 0;
    }
    
    // кнопка поплавок
    if (!keyhs_state && !butths_state && (msHs - btnHsTimer) > 10)
    {
      butths_state = 1;
      butths_long_state = 0;
      btnHsTimer = msHs;
    }
    else if (!keyhs_state && !butths_long_state &&(msHs - btnHsTimer) > 2000)
    {
      butths_long_state = 1;
      // дейтсвие при длительном нажатии
    }
    else if (keyhs_state && butths_state)
    {
      butths_state = 0;
      btnHsTimer = msHs;    
      if(!butths_long_state) // действие при коротком нажатии
      {
	dev_cntrl.float_test = true;    // мерим поле поплавка и смотрим что бы было больше заданного
	SetStatus(st_FLOAT_TEST);
      }
      else 
      {
	SetOnContinuously();            // калибровка катушки под поплавок 
	dev_cntrl.calibrateHS = true;
	SetStatus(st_POPLOVOK_CALIB);
      }
      butths_long_state = 0;
    }
    
    switch(fsm){
    case WAIT : {
      dev_cntrl.cnt1k = 0;
      DisableCoilPower();     
      minR = minRprev = 500000;
      maxHs = maxHSfinale = 0;
      minRiso = minRisoFinale = 0;
      
      if(dev_cntrl.activate_rstest || dev_cntrl.continuously_on){
	EnableCoilPower();
	//  SetRSCurrent(confparam.testparam.Irs);
	osDelay(100);
	fsm = CHOOSING_GAIN;
	nullFlag = 1;
	ClearDSPResult();
      }
      if(dev_cntrl.calibrateR){
	SetCoilPWMFreq(100); 
	SetRSCurrent(50);
	dev_cntrl.stable_on = true;
	fsm = CONTINUOUSLY_ON;
	dev_cntrl.calibrateR = false;
      } 
      if(dev_cntrl.calibrateRiso){
	SetCoilPWMFreq(100); 
	SetRSCurrent(50);
	dev_cntrl.stable_on = true;
	fsm = CONTINUOUSLY_ON;
	dev_cntrl.calibrateRiso = false;
	nullFlag = 1;
      } 
      if(dev_cntrl.float_test){
	led_test_time = LONG_LED;
	led_work_time = LONG_LED;
	SetCoilPWMFreq(100); 
	osDelay(100);   
	dev_cntrl.saved_HS = GetU_HS();
	if (GetCalibHS() < dev_cntrl.max_Hs) LedTest = RED_LED;
	else LedTest = GREEN_LED;
	dev_cntrl.float_test = false;
	SetStatus(st_READY);
	nullFlag = 1;
      }         
      break;
    }
    case CHOOSING_GAIN :{
      SetCoilPWMFreq(100); 
      dev_cntrl.stable_on = true;
      osDelay(50);  
      SetSignalGain(GAIN_1);
      osDelay(50);
      if(GetURSon() > (UREF * 90 / 100)){ /*if more than 90%*/
	dev_cntrl.stable_on = false;
	if(dev_cntrl.continuously_on) fsm = CONTINUOUSLY_ON;
	if(dev_cntrl.activate_rstest) fsm = TEST;
	if(!dev_cntrl.continuously_on && !dev_cntrl.activate_rstest) fsm =WAIT;
	ClearDSPResult();   
	SetSignalGain(GAIN_1);
	SetResistance_corrections(confparam.calibrate.R_offset_GAIN_1,confparam.calibrate.R_gain_GAIN_1); 
	osDelay(50);
	break;
      }
      SetSignalGain(GAIN_2);
      osDelay(50);
      if(GetURSon() > (UREF * 90 / 100)){ /*if more than 90%*/
	dev_cntrl.stable_on = false;
	if(dev_cntrl.continuously_on) fsm = CONTINUOUSLY_ON;
	if(dev_cntrl.activate_rstest) fsm = TEST; 
	if(!dev_cntrl.continuously_on && !dev_cntrl.activate_rstest) fsm =WAIT;
	ClearDSPResult();                      
	SetSignalGain(GAIN_1);
	SetResistance_corrections(confparam.calibrate.R_offset_GAIN_1,confparam.calibrate.R_gain_GAIN_1);            
	osDelay(50);
	break;
      }
      SetSignalGain(GAIN_5);
      osDelay(50);
      if(GetURSon() > (UREF * 90 / 100)){ /*if more than 90%*/
	dev_cntrl.stable_on = false;           
	if(dev_cntrl.continuously_on) fsm = CONTINUOUSLY_ON;
	if(dev_cntrl.activate_rstest) fsm = TEST;  
	if(!dev_cntrl.continuously_on && !dev_cntrl.activate_rstest) fsm =WAIT;
	ClearDSPResult();           
	SetSignalGain(GAIN_2);
	SetResistance_corrections(confparam.calibrate.R_offset_GAIN_2,confparam.calibrate.R_gain_GAIN_2);            
	osDelay(50);
	break;
      }
      SetSignalGain(GAIN_10);
      osDelay(50);
      if(GetURSon() > (UREF * 90 / 100)){ /*if more than 90%*/
	dev_cntrl.stable_on = false;           
	if(dev_cntrl.continuously_on) fsm = CONTINUOUSLY_ON;
	if(dev_cntrl.activate_rstest) fsm = TEST;    
	if(!dev_cntrl.continuously_on && !dev_cntrl.activate_rstest) fsm =WAIT;
	ClearDSPResult();                      
	SetSignalGain(GAIN_5);
	SetResistance_corrections(confparam.calibrate.R_offset_GAIN_5,confparam.calibrate.R_gain_GAIN_5);                     
	osDelay(50);
	break;
      }
      SetSignalGain(GAIN_20);
      osDelay(50);
      if(GetURSon() > (UREF * 90 / 100)){ /*if more than 90%*/
	dev_cntrl.stable_on = false;
	if(dev_cntrl.continuously_on) fsm = CONTINUOUSLY_ON;
	if(dev_cntrl.activate_rstest) fsm = TEST; 
	if(!dev_cntrl.continuously_on && !dev_cntrl.activate_rstest) fsm =WAIT;
	ClearDSPResult();                      
	SetSignalGain(GAIN_10);
	SetResistance_corrections(confparam.calibrate.R_offset_GAIN_10,confparam.calibrate.R_gain_GAIN_10);                      
	osDelay(50);
	break;
      }
      SetSignalGain(GAIN_50);
      osDelay(50);
      if(GetURSon() > (UREF * 90 / 100)){ /*if more than 90%*/
	dev_cntrl.stable_on = false;
	if(dev_cntrl.continuously_on) fsm = CONTINUOUSLY_ON;
	if(dev_cntrl.activate_rstest) fsm = TEST;
	if(!dev_cntrl.continuously_on && !dev_cntrl.activate_rstest) fsm =WAIT;
	ClearDSPResult();           
	SetSignalGain(GAIN_20);
	SetResistance_corrections(confparam.calibrate.R_offset_GAIN_20,confparam.calibrate.R_gain_GAIN_20);                      
	osDelay(50);
	break;
      }
      SetSignalGain(GAIN_100);
      osDelay(50);
      if(GetURSon() > (UREF * 90 / 100)){ /*if more than 90%*/
	dev_cntrl.stable_on = false;
	if(dev_cntrl.continuously_on) fsm = CONTINUOUSLY_ON;
	if(dev_cntrl.activate_rstest) fsm = TEST;   
	if(!dev_cntrl.continuously_on && !dev_cntrl.activate_rstest) fsm =WAIT;
	ClearDSPResult();                      
	SetSignalGain(GAIN_50);
	SetResistance_corrections(confparam.calibrate.R_offset_GAIN_50,confparam.calibrate.R_gain_GAIN_50);                     
	osDelay(50);
	break;
      }
      dev_cntrl.stable_on = false;
      SetResistance_corrections(confparam.calibrate.R_offset_GAIN_100,confparam.calibrate.R_gain_GAIN_100);                   
      SetSignalGain(GAIN_100);         
      if(dev_cntrl.continuously_on) fsm = CONTINUOUSLY_ON;
      if(dev_cntrl.activate_rstest) fsm = TEST;  
      if(!dev_cntrl.continuously_on && !dev_cntrl.activate_rstest) fsm =WAIT;
      ClearDSPResult();
      osDelay(50); 
      break;
    }
    case CONTINUOUSLY_ON : {
      ModuleGetParam(&confparam);
      SetCoilPWMFreq(confparam.testparam.Fcoil);        
      dev_cntrl.saved_HS = GetU_HS();
      dev_cntrl.saved_Riso = GetR_ISO();
      dev_cntrl.saved_RSon = GetRS_ON();     
      
      if ((!dev_cntrl.calibrateR)&& (!dev_cntrl.calibrateRiso)&& (!dev_cntrl.calibrateHS)){
      	if (dev_cntrl.saved_RSon > saved_RSon_old+50 || dev_cntrl.saved_RSon < saved_RSon_old-50 ){
      		saved_RSon_old = dev_cntrl.saved_RSon;
	  	fsm = CHOOSING_GAIN;
      	}
      }
      
      if (dev_cntrl.calibrateHS) // popolovok calib
      {
	if (dev_cntrl.saved_HS != 0 && maxHs < dev_cntrl.saved_HS)
	{
	  maxHs = dev_cntrl.saved_HS; 
	}
	if (maxHs > 740 && (dev_cntrl.saved_HS < (maxHs - 30) || dev_cntrl.saved_HS == 0) && maxHSfinale ==0)
	{
	  maxHSfinale = maxHs - 2;
	  dev_cntrl.TempObserver.inputR = maxHSfinale+800;
	  TempObserverUpdate(&dev_cntrl.TempObserver);
	  SetCoilCurrent(dev_cntrl.TempObserver.outputT);   
	  ModuleGetParam(&confparam);
	  confparam.testparam.Icoil = dev_cntrl.TempObserver.outputT;          
	  ModuleSetParam(&confparam);
	  uint32_t tmp = GetCoilCurrent();
	  if ( confparam.testparam.Icoil != tmp)
	  {
	    SetCoilCurrent(confparam.testparam.Icoil);
	  }
	  
	  timerLed1 = osKernelSysTick();                 
	  LedTest = GREEN_LED;
	}
	if (maxHSfinale !=0 && maxHSfinale == dev_cntrl.saved_HS)
	{
	  dev_cntrl.calibrateHS = false;
	  dev_cntrl.continuously_on = false;
	  dev_cntrl.Status = st_READY; 
	  dev_cntrl.calibrateHS = false;
	}
      } 
      else {
	
	if (dev_cntrl.suzType == SUZ_TOMZEL){     
	  if (dev_cntrl.findMinR)        // ищем наш модуль герконов
	  {
	    
	    if (dev_cntrl.saved_Riso != 0 && minRiso < dev_cntrl.saved_Riso)
	    {
	      minRiso = dev_cntrl.saved_Riso; 
	    }
	    
	    if (minRiso > (confparam.calibrate.R_THREE_GERKON-GERKON_OFFSET)) //  нашли по 2 герконам
	    {
	      minRisoFinale = minRiso;
	      timerLed1 = osKernelSysTick();
	      LedTest = GREEN_LED;
	      dev_cntrl.findMinR = false;
	      dev_cntrl.activate_rstest = false;
	      dev_cntrl.continuously_on = false;
	      dev_cntrl.Status = st_READY; 
	    }
	  }
	  if (dev_cntrl.findR)  // если не сработало все 3 геркона то авария
	  {
	    
	    
	    if (dev_cntrl.saved_Riso >= (confparam.calibrate.R_THREE_GERKON-GERKON_OFFSET))
	    {
	      timerLed1 = osKernelSysTick();
	      LedTest = GREEN_LED;
	    }
	    else
	    {
	      timerLed1 = osKernelSysTick();
	      LedTest = RED_LED;
	    }
	    fsm = WAIT;
	    dev_cntrl.Status = st_READY; 
	    dev_cntrl.findR = false;
	    dev_cntrl.activate_rstest = false;
	    dev_cntrl.continuously_on = false;
	    
	  }
	}
	else{
	  
	  
	}
      }
      
      if(!dev_cntrl.continuously_on) fsm = WAIT;
      break;
    }        
    case TEST : { //  
      ModuleGetParam(&confparam);         
      SetCoilPWMFreq(confparam.testparam.Fcoil);  
      
      if ((!dev_cntrl.calibrateR)&& (!dev_cntrl.calibrateRiso)&& (!dev_cntrl.calibrateHS)){
      	if (dev_cntrl.saved_RSon > saved_RSon_old+50 || dev_cntrl.saved_RSon < saved_RSon_old-50){
      		saved_RSon_old = dev_cntrl.saved_RSon;
	  	fsm = CHOOSING_GAIN;
      	}
      }
      
      if (dev_cntrl.suzType == SUZ_OTHER)
      { 
	if(dev_cntrl.mode == SIGNEMODE){   
	  osDelay(1000);  
	  if (dev_cntrl.cnt1k >= 100){
	    dev_cntrl.saved_Dispertion = GetDispertion();
	    if(dev_cntrl.saved_RSon <= dev_cntrl.max_Rs && dev_cntrl.saved_Dispertion < dev_cntrl.max_Dispersion && dev_cntrl.saved_RSon !=0)  
	    {           
	      timerLed1 = osKernelSysTick();
	      LedTest = GREEN_LED;
	      dev_cntrl.findR = false;
	    }              
	    else   
	    {           
	      timerLed1 = osKernelSysTick();
	      LedTest = RED_LED;
	      dev_cntrl.findR = false;
	    }           
	    fsm = WAIT;
	    dev_cntrl.activate_rstest = false;
	    dev_cntrl.mode = CYCLEMODE;
	    dev_cntrl.Status = st_READY; 
	  }
	}                     
	
	dev_cntrl.saved_RSon = GetRS_ON();      
	
	if (dev_cntrl.findMinR){
	  
	  if (dev_cntrl.saved_RSon != 0 && minR > dev_cntrl.saved_RSon)
	  {
	    minR = dev_cntrl.saved_RSon; //запоминаем минимальное сопротиление
	  }
	  
	  if (minR <10000 && (dev_cntrl.saved_RSon > (minR+1000) || dev_cntrl.saved_RSon == 0))
	  {
	    minRprev = minR;
	    LedTest = GREEN_LED;
	  }
	  if (minRprev < 5000 && InRange(dev_cntrl.saved_RSon, minRprev-10, minRprev+10) && dev_cntrl.saved_RSon !=0) 
	  {
	    dev_cntrl.findMinR = false;
	    dev_cntrl.activate_rstest = false;
	    sqrt(minRprev);
	    dev_cntrl.Status = st_READY; 
	  }
	}
      }
      if(!dev_cntrl.activate_rstest) fsm = WAIT; 
      
      break;
    }        
    }
    osThreadYield();  
  }
}




void SetMaxRs(uint32_t R){
  dev_cntrl.max_Rs = R;
}

void SetMaxDispersion(uint32_t R){
  dev_cntrl.max_Dispersion = R;
}

void SetMaxHs(uint32_t R){
  dev_cntrl.max_Hs = R;
}

void StartMagnitTest(){
  dev_cntrl.Status = st_FLOAT_TEST;
  dev_cntrl.float_test = true;
}

void SetSignleTestMode(){
  dev_cntrl.Status = st_SIGNEMODE;
  dev_cntrl.mode = SIGNEMODE;  
}

void SetCycleTestMode(){
  dev_cntrl.Status = st_CYCLEMODE;
  dev_cntrl.mode = CYCLEMODE;   
}

void StartRSTest(){
  dev_cntrl.activate_rstest = true; 
}

void StopRSTest(){
  dev_cntrl.activate_rstest = false; 
  dev_cntrl.continuously_on = false; 
  dev_cntrl.Status = st_READY; 
}

void StartRISOTest(){
  dev_cntrl.RISO_test = true;
}

void StopRISOTest(){
  dev_cntrl.RISO_test = false;
}

void SetOnContinuously(){
  dev_cntrl.continuously_on = true; 
}



uint16_t Calibration(uint8_t gain){
  uint16_t r;
  
  SetSignalGain(gain);
  dev_cntrl.calibrateR = true;
  osDelay(1000);
  r = GetRS_ON_clear();
  StopRSTest();
  return r;
}

uint16_t CalibRIso(uint8_t gain){
  
  uint16_t r;
  
  SetSignalGain(gain);
  dev_cntrl.calibrateRiso = true;
  osDelay(1000);
  r = GetR_ISO();
  StopRSTest();
  return r;
}

bool IsContinuouslyMode(){
  return (dev_cntrl.continuously_on || dev_cntrl.stable_on); 
}

uint32_t GetLastRSon(){
  
  if (dev_cntrl.suzType == SUZ_TOMZEL) return 0;
  return dev_cntrl.saved_RSon; 
}

uint32_t GetSuzType(){
  return dev_cntrl.suzType; 
}

uint32_t GetStatus(){
  return dev_cntrl.Status; 
}

uint32_t GetConnectStatus(){
  return dev_cntrl.connect_on; 
}

void SetStatus(TStatusType s){
  dev_cntrl.Status = s;
}


uint32_t GetSavedRIso(){
  
  if (dev_cntrl.suzType == SUZ_TOMZEL){  
  //  if(dev_cntrl.saved_Riso > (confparam.calibrate.R_THREE_GERKON-GERKON_OFFSET)) return THREE_GERKON;
    if (InRange(dev_cntrl.saved_Riso, confparam.calibrate.R_THREE_GERKON-GERKON_OFFSET, confparam.calibrate.R_THREE_GERKON+GERKON_OFFSET)) return THREE_GERKON;
    else if (InRange(dev_cntrl.saved_Riso, confparam.calibrate.R_TWO_GERKON-GERKON_OFFSET, confparam.calibrate.R_TWO_GERKON+GERKON_OFFSET)) return TWO_GERKON;
    else if (InRange(dev_cntrl.saved_Riso, confparam.calibrate.R_ONE_GERKON-GERKON_OFFSET, confparam.calibrate.R_ONE_GERKON+GERKON_OFFSET)) return ONE_GERKON;
    else return 11;
  }
  else return 0;
}

double GetSavedDispertion(){
 return dev_cntrl.saved_Dispertion;
}

void SetSuzType(uint32_t suz){
  
  switch(suz){
  case 0 : { // наш 
    dev_cntrl.suzType = SUZ_TOMZEL;
    break;
  }
  case 1 : { // сторонники
    dev_cntrl.suzType = SUZ_OTHER;
    break;
  }
  }
}

void SetFindR(bool findR)
{
	dev_cntrl.findR = findR;
}

uint16_t GetCalibHS(){
  
  uint16_t HS = GetU_HS();
  uint16_t HSoffset = (uint16_t)confparam.calibrate.HS_offset;
  if (HS > HSoffset) return abs(HS - HSoffset);
  else return 0;
  
}
