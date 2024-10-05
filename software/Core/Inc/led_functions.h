/*
 * led_functions.h
 *
 *  Created on: Jan 24, 2024
 *      Author: Dane
 */

#ifndef LED_FUNCTIONS_H_
#define LED_FUNCTIONS_H_

#define ROWS 8
#define COLUMNS 11
#define ON_LEDS 3		// Number of LEDs to leave on at a time?
#define NUM_PATTERNS 2

// Used for determining pattern timer frequency
static const uint8_t pattern_steps[3] = {10, 10, 10};

void TurnOnLED(uint8_t row, uint8_t col);
void TurnOffLED(uint8_t row, uint8_t col);
void SetLEDQueue( uint8_t (*led_list)[][2], uint8_t len );
void AddToLEDQueue( uint8_t (*led_list)[][2], uint8_t len );
void HandleLEDQueue();
void PulseHandler();
void PulseHandlerKeepOn();
void PrintRate( uint8_t rate );
void TurnAllOff();
void ResetIndexes();

#endif /* LED_FUNCTIONS_H_ */
