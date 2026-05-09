#include "rtds.h"

#include <tx_api.h>

#include "io.h"

/**
 * @brief       Activates the RTDS and sleeps the calling thread for the
 *              duration of activation
 *
 * @param[in]   config_ptr  RTDS configuration
 */
status_t rtds_activate(const config_rtds_t* config_ptr)
{
    VCU_Output_High(config_ptr->port, config_ptr->pin);
    LOG_INFO("Waiting - RTDS\n");
    UINT tx_status = tx_thread_sleep(config_ptr->active_ticks);
    LOG_INFO("Waited - RTDS\n");
    VCU_Output_Low(config_ptr->port, config_ptr->pin);

    return (tx_status == TX_SUCCESS) ? STATUS_OK : STATUS_ERROR;
}