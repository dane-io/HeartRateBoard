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

// Define LEDs for 0-9 tens digit, ones digit, and 1 for 100
// Ones digit is the same with columns + 4 (except for 2, added extras to make compatible)
uint8_t zero_tens[][2] = {{2,2},{3,2},{4,2},{1,3},{5,3},{2,4},{3,4},{4,4}};
uint8_t one_tens[][2] = {{2,2},{1,3},{2,3},{3,3},{4,3},{5,3}};
uint8_t two_tens[][2] = {{1,2},{1,3},{2,4},{3,3},{4,2},{5,2},{5,3},{5,4}};
uint8_t three_tens[][2] = {{1,2},{1,3},{2,4},{3,3},{4,4},{5,3},{5,2}};
uint8_t four_tens[][2] = {{1,2},{1,4},{2,2},{2,4},{3,2},{3,3},{3,4},{4,4},{5,4}};
uint8_t five_tens[][2] = {{1,2},{1,3},{1,4},{2,2},{3,2},{3,3},{3,4},{4,4},{5,3},{5,2}};
uint8_t six_tens[][2] = {{1,3},{1,4},{2,2},{3,2},{3,3},{3,4},{4,2},{4,4},{5,3}};
uint8_t seven_tens[][2] = {{1,2},{1,3},{1,4},{2,4},{3,3},{4,2}};
uint8_t eight_tens[][2] = {{1,2},{1,3},{1,4},{2,2},{2,4},{3,2},{3,3},{3,4},{4,2},{4,4},{5,2},{5,3},{5,4}};
uint8_t nine_tens[][2] = {{1,2},{1,3},{1,4},{2,2},{2,4},{3,2},{3,3},{3,4},{4,4},{5,2},{5,3},{5,4}};

uint8_t zero_ones[][2] = {{2,6},{3,6},{4,6},{1,7},{5,7},{2,8},{3,8},{4,8}};
uint8_t one_ones[][2] = {{2,6},{1,7},{2,7},{3,7},{4,7},{5,7}};
uint8_t two_ones[][2] = {{1,6},{1,7},{2,8},{3,7},{4,6},{5,6},{5,7},{5,8}};
uint8_t three_ones[][2] = {{1,6},{1,7},{2,8},{3,7},{4,8},{5,7},{5,6}};
uint8_t four_ones[][2] = {{1,6},{1,8},{2,6},{2,8},{3,6},{3,7},{3,8},{4,8},{5,8}};
uint8_t five_ones[][2] = {{1,6},{1,7},{1,8},{2,6},{3,6},{3,7},{3,8},{4,8},{5,7},{5,6}};
uint8_t six_ones[][2] = {{1,7},{1,8},{2,6},{3,6},{3,7},{3,8},{4,6},{4,8},{5,7}};
uint8_t seven_ones[][2] = {{1,6},{1,7},{1,8},{2,8},{3,7},{4,6}};
uint8_t eight_ones[][2] = {{1,6},{1,7},{1,8},{2,6},{2,8},{3,6},{3,7},{3,8},{4,6},{4,8},{5,6},{5,7},{5,8}};
uint8_t nine_ones[][2] = {{1,6},{1,7},{1,8},{2,6},{2,8},{3,6},{3,7},{3,8},{4,8},{5,6},{5,7},{5,8}};

uint8_t one_hundreds[][2] = {{1,0},{2,0}};
uint8_t* tens_digits[10] = {&zero_tens, &one_tens, &two_tens, &three_tens, &four_tens, &five_tens, &six_tens, &seven_tens, &eight_tens, &nine_tens};
uint8_t* ones_digits[10] = {&zero_ones, &one_ones, &two_ones, &three_ones, &four_ones, &five_ones, &six_ones, &seven_ones, &eight_ones, &nine_ones};
uint8_t digit_sizes[10] = {8, 6, 8, 7, 9, 10, 9, 6, 13, 12};
#endif /* LED_PATTERNS_H_ */

