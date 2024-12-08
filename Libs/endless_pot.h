#ifndef ENDLESS_POT_H
#define ENDLESS_POT_H

#include <stdint.h>

#define ADC_HYSTERESIS 250 // Hysteresis value for ADC readings

typedef struct {
	uint8_t state; // BIT0 forwards, BIT1 backwards, BIT2 medium, BIT3 fast, BIT7 error
	uint16_t angle; // absolute position 0..65535
	uint32_t last_update; // timestamp of last update
} endless_pot_t;

void endless_pot_init(endless_pot_t* pot);
void endless_pot_update(endless_pot_t* pot, uint16_t adc_a, uint16_t adc_b);

#endif // ENDLESS_POT_H