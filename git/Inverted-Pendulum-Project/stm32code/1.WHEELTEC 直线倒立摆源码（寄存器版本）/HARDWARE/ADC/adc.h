#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
 /**************************************************************************
作者：平衡小车之家 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/
#define Battery_Ch 6
u16 Get_Adc(u8 ch);
u16 Get_Adc2(u8 ch);
int Get_battery_volt(void);   
void Adc_Init(void);  
u16 Get_Adc_Average(u8 ch,u8 times);
#endif 















