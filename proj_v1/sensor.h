/*
 * sensor.h
 *
 *  Created on: Apr 2, 2021
 *      Author: Yu L
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include <msp.h>
#include <stdio.h>

#define TICKS ((uint16_t)0xB3B0)
#define TICKS2 ((uint16_t)0xB3A8)   //output pulse width equals period(Ticks)-period(Ticks2), if ticks or ticks2 were loaded into CCR[0] during up mode

void config_pulse_timer(void);

void start_pulse(void);

void stop_pulse(void);

void config_capture_timer(void);

void config_gpio(void);

#endif /* SENSOR_H_ */
