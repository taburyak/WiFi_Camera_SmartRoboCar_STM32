/*
 * File: HCSR04_cfg.c
 * Driver Name: [[ HC-SR04 Ultrasonic Sensor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "HCSR04.h"

const HCSR04_CfgType HCSR04_CfgParam[HCSR04_UNITS] =
{
	// HC-SR04 Sensor Unit 1 Configurations
    {
		ECHO_TRIG_FRONT_GPIO_Port,
		ECHO_TRIG_FRONT_Pin,
		TIM2,
		TIM_CHANNEL_1,
		100
	},
	{
		ECHO_TRIG_BACK_GPIO_Port,
		ECHO_TRIG_BACK_Pin,
		TIM5,
		TIM_CHANNEL_1,
		100
	}
};
