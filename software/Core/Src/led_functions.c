/*
 * led_patterns.c
 *
 *  Created on: Jan 24, 2024
 *      Author: Dane
 */


#include "main.h"
#include "led_functions.h"
#include "led_patterns.h"
#include <string.h>


TIM_HandleTypeDef* cathode_timers[ROWS];
uint32_t cathode_channels[ROWS] = {TIM_CHANNEL_1,
										  TIM_CHANNEL_2,
										  TIM_CHANNEL_3,
										  TIM_CHANNEL_4,
										  TIM_CHANNEL_1,
										  TIM_CHANNEL_2,
										  TIM_CHANNEL_3,
										  TIM_CHANNEL_4};


GPIO_TypeDef* cathode_ports[ROWS] = {C0_GPIO_Port,
									 C1_GPIO_Port,
									 C2_GPIO_Port,
									 C3_GPIO_Port,
									 C4_GPIO_Port,
									 C5_GPIO_Port,
									 C6_GPIO_Port,
									 C7_GPIO_Port};
uint16_t cathode_pins[ROWS] = {C0_Pin,
							   C1_Pin,
							   C2_Pin,
							   C3_Pin,
							   C4_Pin,
							   C5_Pin,
							   C6_Pin,
							   C7_Pin};


GPIO_TypeDef* anode_ports[COLUMNS] = {A0_GPIO_Port,
											 A1_GPIO_Port,
											 A2_GPIO_Port,
											 A3_GPIO_Port,
											 A4_GPIO_Port,
											 A5_GPIO_Port,
											 A6_GPIO_Port,
											 A7_GPIO_Port,
											 A8_GPIO_Port,
											 A9_GPIO_Port,
											 A10_GPIO_Port};
uint16_t anode_pins[COLUMNS] = {A0_Pin,
									   A1_Pin,
									   A2_Pin,
									   A3_Pin,
									   A4_Pin,
									   A5_Pin,
									   A6_Pin,
									   A7_Pin,
									   A8_Pin,
									   A9_Pin,
									   A10_Pin};

void TurnOnLED(uint8_t row, uint8_t col) {
	HAL_GPIO_WritePin(anode_ports[col], anode_pins[col], GPIO_PIN_SET);
	HAL_GPIO_WritePin(cathode_ports[row], cathode_pins[row], GPIO_PIN_RESET);
	//HAL_TIM_PWM_Start(cathode_timers[row], cathode_channels[row]);
}

void TurnOffLED(uint8_t row, uint8_t col) {
	HAL_GPIO_WritePin(anode_ports[col], anode_pins[col], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(cathode_ports[row], cathode_pins[row], GPIO_PIN_SET);
	//HAL_TIM_PWM_Stop(cathode_timers[row], cathode_channels[row]);
}




uint8_t led_queue[ROWS*COLUMNS+1][2] = {};	// End of queue is 0xFF (255)
//led_queue[ROWS*COLUMNS][0] = 0xFF;
volatile uint8_t queue_index = 0;
uint8_t queue_len;

// Make led_queue equal to led_list
void SetLEDQueue( uint8_t (*led_list)[][2], uint8_t len ) {
	memcpy(led_queue, led_list, len*2);
	//queue_len = sizeof(led_list)/2;
	queue_len = len;
	//led_queue[len][0] = 0xFF;
	//led_queue[len][1] = 0xFF;
	queue_index = 0;

}

// Concatenate led_list to led_queue
void AddToLEDQueue( uint8_t (*led_list)[][2], uint8_t len ) {
	memcpy(led_queue+queue_len, led_list, len*2);
	queue_len += len;
	//queue_index = 0;
}

void HandleLEDQueue() {

	if (queue_len > 0) {	// Don't do anything unless there's something in the queue
		if (queue_index <= 0) {
			TurnOffLED(led_queue[queue_index-1+queue_len][0], led_queue[queue_index-1+queue_len][1]);
		}
		else {
			TurnOffLED(led_queue[queue_index-1][0], led_queue[queue_index-1][1]);
		}

		TurnOnLED(led_queue[queue_index][0], led_queue[queue_index][1]);

		queue_index++;
		if (queue_index >= queue_len) {
			queue_index = 0;
		}
	}

}


volatile uint8_t pulse_step = 0;
volatile int8_t pulse_dir = 1;
void PulseHandler() {
	TurnAllOff();
	SetLEDQueue(pulse_step_list[pulse_step], pulse_step_sizes[pulse_step]);
	pulse_step += pulse_dir;
	if (pulse_step >= PULSE_STEPS) {
		pulse_step--;
		pulse_dir = -1;
	}
	else if (pulse_step == 0) {
		pulse_dir = 1;
	}
}

void PulseHandlerKeepOn() {
	TurnAllOff();
	if (pulse_step <= 0) {
		SetLEDQueue(pulse_step_list[0], pulse_step_sizes[0]);
		pulse_dir = 1;
		pulse_step = 1;
	}
	// Added this to extend last step by another cycle
	else if (pulse_step >= PULSE_STEPS) {
		pulse_step = PULSE_STEPS - 1;
		pulse_dir = -1;
	}
	else if (pulse_dir > 0) {
		AddToLEDQueue(pulse_step_list[pulse_step], pulse_step_sizes[pulse_step]);
		pulse_step++;
		//if (pulse_step >= PULSE_STEPS) {
		//	pulse_step--;
		//	pulse_dir = -1;
		//}
	}
	else {
		queue_len -= pulse_step_sizes[pulse_step];
		queue_index = 0;
		pulse_step--;
	}
}

void PrintRate( uint8_t rate ) {

	// Find tens, and ones digit separately (hundreds is either 1 or 0)
	uint8_t tens = (rate % 100) / 10;
	uint8_t ones = (rate % 100) % 10;

	//queue_len = 0;	// Reset queue?
	ResetIndexes();
	TurnAllOff();

	if (rate >= 100) {
		AddToLEDQueue(&one_hundreds, 2);
	}
	AddToLEDQueue(tens_digits[tens], digit_sizes[tens]);
	AddToLEDQueue(ones_digits[ones], digit_sizes[ones]);

}

void TurnAllOff() {
	for (uint8_t i = 0; i < queue_len; i++) {
		TurnOffLED(led_queue[i][0], led_queue[i][1]);
	}
}

void ResetIndexes() {
	TurnAllOff();
	queue_len = 0;
	queue_index = 0;
	pulse_dir = 1;
	pulse_step = 0;
}

