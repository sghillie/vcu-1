#include "wheelspeed.h"

#include <can_s.h>
#include "main.h"
#include "log.h"

static void wheelspeed_thread_entry(ULONG input);
static void wheelspeed_timer_callback(ULONG input);
static void compute_and_broadcast(wheelspeed_context_t *wh);
static float compute_wheel_rpm(wheel_state_t *wheel, const config_wheelspeed_t *config);

status_t wheelspeed_init(wheelspeed_context_t *wh,
                         rtcan_handle_t *rtcan_h,
                         TX_BYTE_POOL *stack_pool_ptr,
                         const config_wheelspeed_t *config_ptr)
{
    wh->config_ptr = config_ptr;
    wh->rtcan_h = rtcan_h;

    void *stack_ptr = NULL;
    UINT tx_status = tx_byte_allocate(stack_pool_ptr,
                                      &stack_ptr,
                                      config_ptr->thread.stack_size,
                                      TX_NO_WAIT);

    if (tx_status == TX_SUCCESS)
    {
        tx_status = tx_semaphore_create(&wh->sample_semaphore,
                                        (CHAR *)"wheelspeed_sem",
                                        0);
    }
    
    if (tx_status == TX_SUCCESS)
    {
        tx_status = tx_timer_create(&wh->sample_timer,
                                    (CHAR *)"wheelspeed_timer",
                                    wheelspeed_timer_callback,
                                    (ULONG)wh,
                                    config_ptr->sample_period_ticks,
                                    config_ptr->sample_period_ticks,
                                    TX_AUTO_ACTIVATE);
    }

    if (tx_status == TX_SUCCESS)
    {
        tx_status = tx_thread_create(&wh->thread,
                                     (CHAR *)config_ptr->thread.name,
                                     wheelspeed_thread_entry,
                                     (ULONG)wh,
                                     stack_ptr,
                                     config_ptr->thread.stack_size,
                                     config_ptr->thread.priority,
                                     config_ptr->thread.priority,
                                     TX_NO_TIME_SLICE,
                                     TX_AUTO_START);
    }

    return (tx_status == TX_SUCCESS) ? STATUS_OK : STATUS_ERROR;
}

static void wheelspeed_timer_callback(ULONG input)
{
    wheelspeed_context_t *wh = (wheelspeed_context_t *)input;
    tx_semaphore_put(&wh->sample_semaphore);
}

static void wheelspeed_thread_entry(ULONG input)
{
    wheelspeed_context_t *wh = (wheelspeed_context_t *)input;

    while (1)
    {
        tx_semaphore_get(&wh->sample_semaphore, TX_WAIT_FOREVER);
        compute_and_broadcast(wh);
    }
}

static float compute_wheel_rpm(wheel_state_t *wheel, const config_wheelspeed_t *config)
{
    uint32_t current = wheel->isr_count;
    uint32_t delta = current - wheel->prev_count;
    wheel->prev_count = current;

    float revs_per_second = (float)delta * TX_TIMER_TICKS_PER_SECOND
                             / ((float)config->ticks_per_wheel
                                * (float)config->sample_period_ticks);
    return revs_per_second * 60.0;
}

static void compute_and_broadcast(wheelspeed_context_t *wh)
{
    const config_wheelspeed_t *cfg = wh->config_ptr;

    float rpm_fr = compute_wheel_rpm(&wh->wheel_fr, cfg);
    float rpm_fl = compute_wheel_rpm(&wh->wheel_fl, cfg);
    float rpm_rr = compute_wheel_rpm(&wh->wheel_rr, cfg);
    float rpm_rl = compute_wheel_rpm(&wh->wheel_rl, cfg);

    {
        struct can_s_wheel_rpm_t data = {
            .wheel_fr_rpm = can_s_wheel_rpm_wheel_fr_rpm_encode(rpm_fr),
            .wheel_fl_rpm = can_s_wheel_rpm_wheel_fl_rpm_encode(rpm_fl),
            .wheel_rr_rpm = can_s_wheel_rpm_wheel_rr_rpm_encode(rpm_rr),
            .wheel_rl_rpm = can_s_wheel_rpm_wheel_rl_rpm_encode(rpm_rl),
        };
        rtcan_msg_t msg = {
            .identifier = CAN_S_WHEEL_RPM_FRAME_ID,
            .length = CAN_S_WHEEL_RPM_LENGTH,
            .extended = CAN_S_WHEEL_RPM_IS_EXTENDED,
        };
        can_s_wheel_rpm_pack(msg.data, &data, msg.length);
        if (rtcan_transmit(wh->rtcan_h, &msg) != RTCAN_OK)
            LOG_ERROR("wheelspeed: failed to transmit RPM\n");
    }

    {
        struct can_s_wheel_speeds_t data = {
            .wheel_fr_speed = can_s_wheel_speeds_wheel_fr_speed_encode(rpm_fr / 60.0 * cfg->wheel_circumference_meters),
            .wheel_fl_speed = can_s_wheel_speeds_wheel_fl_speed_encode(rpm_fl / 60.0 * cfg->wheel_circumference_meters),
            .wheel_rr_speed = can_s_wheel_speeds_wheel_rr_speed_encode(rpm_rr / 60.0 * cfg->wheel_circumference_meters),
            .wheel_rl_speed = can_s_wheel_speeds_wheel_rl_speed_encode(rpm_rl / 60.0 * cfg->wheel_circumference_meters),
        };
        rtcan_msg_t msg = {
            .identifier = CAN_S_WHEEL_SPEEDS_FRAME_ID,
            .length = CAN_S_WHEEL_SPEEDS_LENGTH,
            .extended = CAN_S_WHEEL_SPEEDS_IS_EXTENDED,
        };
        can_s_wheel_speeds_pack(msg.data, &data, msg.length);
        if (rtcan_transmit(wh->rtcan_h, &msg) != RTCAN_OK)
            LOG_ERROR("wheelspeed: failed to transmit speeds\n");
    }
}

void wheelspeed_handle_exti(wheelspeed_context_t *wh, uint16_t gpio_pin)
{
    if (gpio_pin == WHEELSPEED_FR_Pin)
        wh->wheel_fr.isr_count++;
    else if (gpio_pin == WHEELSPEED_FL_Pin)
        wh->wheel_fl.isr_count++;
    else if (gpio_pin == WHEELSPEED_RR_Pin)
        wh->wheel_rr.isr_count++;
    else if (gpio_pin == WHEELSPEED_RL_Pin)
        wh->wheel_rl.isr_count++;
}
