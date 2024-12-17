#include "endless_pot.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32f0xx_hal.h>

#define M_PI_F ((float)M_PI)

void endless_pot_init(endless_pot_t* pot){
    pot->state = 0;
    pot->angle = 0;
    pot->last_update = 0;
    pot->mean_a_N = 0;
    pot->mean_b_N = 0;
    pot->confidence = 0;
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

void endless_pot_update(endless_pot_t* pot, uint16_t adc_a, uint16_t adc_b){
    // filter the ADC readings
    /*
    pot->mean_a_N = pot->mean_a_N + adc_a - (pot->mean_a_N / RUNNING_AVERAGE_SIZE);
    pot->mean_b_N = pot->mean_b_N + adc_b - (pot->mean_b_N / RUNNING_AVERAGE_SIZE);
    adc_a = pot->mean_a_N / RUNNING_AVERAGE_SIZE;
    adc_b = pot->mean_b_N / RUNNING_AVERAGE_SIZE;
    */

    // normalize the ADC readings
    float a = (adc_a - 2047.5f) / 2047.5f;
    float b = (adc_b - 2047.5f) / 2047.5f;
    //float current_angle = atan2f(b, a);

    // Calculate the current angle
    float current_angle = fast_atan2f(b, a);

    int16_t current_angle_fixed = (int16_t)(current_angle * 32768.0f / M_PI_F);

    // Calculate the difference between the current and previous angle
    int32_t angle_diff = current_angle_fixed - (int16_t)pot->angle;

    // Apply hysteresis
    if (abs(angle_diff) > ADC_HYSTERESIS){
        pot->confidence++;
        if (pot->confidence < 2) return;
        pot->angle = (uint16_t)current_angle_fixed;
        uint32_t current_time = HAL_GetTick();
        uint32_t time_diff = current_time - pot->last_update;
        pot->state = 0;
        // Determine angle speed is angle per time
/*
        if (mean_ad > 7500){
            pot->state |= 0x08;
        }
        /*else if (mean_ad > 200){
            pot->state |= 0x04;
        }
        */
        // Determine rotation direction
        if (angle_diff > 0){
            pot->state |= 0x01;
        }
        else{
            pot->state |= 0x02;
        }
        pot->last_update = current_time;
    }else{
        pot->confidence = 0;
        pot->state = 0x00;
    }
}
