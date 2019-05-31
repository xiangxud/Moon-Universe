#ifndef __BOARD_H
#define __BOARD_H

#include "stm32f4xx.h"
#include "explore_system.h"
#include "explore_systick.h"
#include "SPI1.h"
#include "SPI3.h"
#include "Task.h"
#include "mcpwm.h"
#include "Flash.h"
#include "Adc_Battery.h"
#include "General_Gpio.h"
#include "Usart3toBluetooth.h"
#include "Usart1toOnboardPC.h"
#include "Uart4toTOF.h"
#include "Sensor.h"
#include "UsbDriver.h"
#include "MemoryMessage.h"

//定义硬件飞控板
void Board_Init(void);

#define BirdFlight            //定义飞控型号 BirdFlight  SpeedyBeeF4
#define T_Motor              	 //定义电机动力 T_Motor     Hoppywing

#endif

