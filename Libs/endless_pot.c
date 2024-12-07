#include "endless_pot.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define M_PI_F ((float)M_PI)

void endless_pot_init(EndlessPot_t* pot){
    pot->state = 0;
    pot->angle = 0;
}

float fast_atan2f(float y, float x) {
    if (x == 0.0f) {
        if (y > 0.0f) return M_PI_F / 2.0f;
        if (y == 0.0f) return 0.0f;
        return -M_PI_F / 2.0f;
    }
    float atan;
    float z = y / x;
    if (fabsf(z) < 1.0f) {
        atan = z / (1.0f + 0.28f * z * z);
        if (x < 0.0f) {
            if (y < 0.0f) return atan - M_PI_F;
            return atan + M_PI_F;
        }
    } else {
        atan = M_PI_F / 2.0f - z / (z * z + 0.28f);
        if (y < 0.0f) return atan - M_PI_F;
    }
    return atan;
}

void endless_pot_update(EndlessPot_t* pot, uint16_t adc_a, uint16_t adc_b){
    float a = (adc_a - 2047.5f) / 2047.5f;
    float b = (adc_b - 2047.5f) / 2047.5f;
    //float current_angle = atan2f(b, a);
    float current_angle = fast_atan2f(b, a);

    int16_t current_angle_fixed = (int16_t)(current_angle * 32768.0f / M_PI_F);

    // Calculate the difference between the current and previous angle
    int32_t angle_diff = current_angle_fixed - (int16_t)pot->angle;


    // Apply hysteresis
    if (abs(angle_diff) > ADC_HYSTERESIS){
        pot->angle = (uint16_t)current_angle_fixed;
        // Determine rotation direction
        if (angle_diff > 0){
            pot->state = 0x01;
        }
        else{
            pot->state = 0x02;
        }
    }else{
        pot->state = 0x00;
    }
}
