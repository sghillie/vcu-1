/***************************************************************************
 * @file   io.h
 * @author Robert Kirkbride (@r-kirkbride, rgak1g24@soton.ac.uk)
 * @brief  I/O interface
 ***************************************************************************/

#ifndef IO_H
#define IO_H

#include "gpio.h"
#include <stdint.h>

#include "status.h"

status_t VCU_Output_High(GPIO_TypeDef* port, uint16_t pin);
status_t VCU_Output_Low(GPIO_TypeDef* port, uint16_t pin);
status_t VCU_Output_Write(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state);
status_t VCU_Output_Toggle(GPIO_TypeDef* port, uint16_t pin);

#endif // IO_H