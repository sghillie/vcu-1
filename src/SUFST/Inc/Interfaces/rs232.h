#ifndef RS232_H
#define RS232_H

#include <stdbool.h>

void rs232_enable(void);
void rs232_disable(void);
bool rs232_is_valid(void);

#endif
