#ifndef ENDLESS_POT_H
#define ENDLESS_POT_H

#include <stdint.h>

#define ENDLESS_POT_RESOLUTION 10 // Resolution of the endless potentiometer in bits
#define ADC_HYSTERESIS (1 << (16 - ENDLESS_POT_RESOLUTION)) // Hysteresis value for ADC readings
#define RUNNING_AVERAGE_SIZE (1 << 2) // Size of the running average filter, power of twos
#define CONFIDENCE_THRESHOLD 2 // Confidence threshold for the potentiometer

typedef struct {
	uint8_t state; // BIT0 forwards, BIT1 backwards, BIT2 medium, BIT3 fast, BIT7 error
	uint16_t angle; // absolute position 0..65535
	uint32_t last_update; // timestamp of last update
	int32_t mean_a_N, mean_b_N; // running average filter
	uint8_t confidence;
} endless_pot_t;

void endless_pot_init(endless_pot_t* pot);
void endless_pot_update(endless_pot_t* pot, uint16_t adc_a, uint16_t adc_b);

#endif // ENDLESS_POT_H