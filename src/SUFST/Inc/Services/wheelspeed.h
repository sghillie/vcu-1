/******************************************************************************
 * @file    wheelspeed.h
 * @author  Martin Perreau (@maartin0)
 * @brief   Wheelspeed interrupt reader for 4 wheels
 * @details Computes wheelspeeds on the fly for 4 wheelspeed sensors, 
            needs to handle ~4000 interrupts/second
            and broadcasts results in both RPM and m/s over CAN-S
 *****************************************************************************/

#ifndef WHEELSPEED_H
#define WHEELSPEED_H

#include <stdint.h>
#include <tx_api.h>
#include <rtcan.h>

#include "config.h"
#include "status.h"

typedef struct
{
    volatile uint32_t isr_count; // live count updated by ISR
    uint32_t prev_count;         // snapshot from last sample
} wheel_state_t;

typedef struct
{
    TX_THREAD thread;
    TX_SEMAPHORE sample_semaphore;
    TX_TIMER sample_timer;
    rtcan_handle_t *rtcan_h;
    wheel_state_t wheel_fr;
    wheel_state_t wheel_fl;
    wheel_state_t wheel_rr;
    wheel_state_t wheel_rl;
    const config_wheelspeed_t *config_ptr;
} wheelspeed_context_t;

status_t wheelspeed_init(wheelspeed_context_t *wheelspeed_h,
                         rtcan_handle_t *rtcan_h,
                         TX_BYTE_POOL *stack_pool_ptr,
                         const config_wheelspeed_t *config_ptr);

void wheelspeed_handle_exti(wheelspeed_context_t *wheelspeed_h, uint16_t gpio_pin);

#endif
