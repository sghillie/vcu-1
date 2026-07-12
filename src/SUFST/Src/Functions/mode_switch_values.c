#include "mode_switch_values.h"

ctrl_mode_t convert_mode_adc_to_discrete(uint16_t adc_reading_0_330)
{
    if (adc_reading_0_330 < 15)
        return 1;
    if (adc_reading_0_330 < 45)
        return 2;
    if (adc_reading_0_330 < 75)
        return 3;
    if (adc_reading_0_330 < 105)
        return 4;
    if (adc_reading_0_330 < 135)
        return 5;
    if (adc_reading_0_330 < 165)
        return 6;
    if (adc_reading_0_330 < 195)
        return 7;
    if (adc_reading_0_330 < 225)
        return 8;
    if (adc_reading_0_330 < 255)
        return 9;
    if (adc_reading_0_330 < 285)
        return 10;
    if (adc_reading_0_330 < 315)
        return 11;
    if (adc_reading_0_330 < 345)
        return 12;
    return 0;
}
