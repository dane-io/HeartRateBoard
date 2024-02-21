/*
 * led_patterns.h
 *
 *  Created on: Jan 24, 2024
 *      Author: Dane
 */

#ifndef LED_PATTERNS_H_
#define LED_PATTERNS_H_

#define PULSE_STEPS 5
uint8_t pulse_step0[][2] = {{3,5}};
uint8_t pulse_step1[][2] = {{2,5},{4,5},{3,4},{3,6}};
uint8_t pulse_step2[][2] = {{2,4},{3,3},{4,4},{5,5},{4,6},{3,7},{2,6},{2,5}};
uint8_t pulse_step3[][2] = {{2,1},{1,2},{2,2},{3,2},{1,3},{2,3},{4,3},{1,4},{5,4},{6,5},{1,6},{5,6},{1,7},{2,7},{4,7},{1,8},{2,8},{3,8},{2,9},{2,5}};
uint8_t pulse_step4[][2] = {{1,0},{2,0},{0,1},{1,1},{3,1},{0,2},{4,2},{0,3},{5,3},{6,4},{7,5},{6,6},{0,7},{5,7},{0,8},{4,8},{0,9},{1,9},{3,9},{1,10},{2,10},{2,5},{1,4},{1,6}};
uint8_t* pulse_step_list[PULSE_STEPS] = {&pulse_step0, &pulse_step1, &pulse_step2, &pulse_step3, &pulse_step4};
uint8_t pulse_step_sizes[PULSE_STEPS] = {1, 4, 8, 20, 24};

#endif /* LED_PATTERNS_H_ */

