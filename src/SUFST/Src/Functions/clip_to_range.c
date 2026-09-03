#include "clip_to_range.h"

/**
 * @brief       Clips a value to a range
 *
 * @details     Value is clipped to [min, max]
 *
 * @param[in]   value   Value to clip
 * @param[in]   min     Minimum of range
 * @param[in]   max     Maximum of range
 * Syntax Explanation
 * condition ? expression-true : expression-false
 * condition ? expression-true : condition ? expression-true : expression-false
 */
uint16_t clip_to_range(uint16_t value, uint16_t min, uint16_t max)
{
    return value < min ? min : value > max ? max : value;
}

uint16_t clip_to_range_shifted_and_scaled(uint16_t value, float scalar, uint16_t min, uint16_t max)
{
    // scalar * (clipped - min)    
    return = (uint16_t)(scalar * (float)(clip_to_range(value, min, max) - min));
}
