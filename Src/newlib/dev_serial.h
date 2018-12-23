//Definitions of the serial devices in the project.
//The stuff here interfaces our native implementation with the device manager
//we created for gluing onto newlib.
#ifndef __DEV_SERIAL_H
#define __DEV_SERIAL_H

#include "newlib_device.h"


//XXX could use the device-specific instance structure to factor this
//implementation


//our 'ttys0' device on USART6
extern const DM_DEVICE g_devUSART6_if;

//our 'ttys4' device on USB
extern const DM_DEVICE g_devCDC_if;



#endif
