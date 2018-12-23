//Definitions of the standard IO devices in the project.
//The stuff here interfaces our native implementation with the device manager
//we created for gluing onto newlib.
#ifndef __DEV_STDIO_H
#define __DEV_STDIO_H

#include "newlib_device.h"


//this thunks over to whatever implemented stdin, stdout, stderr
extern const DM_DEVICE g_devStdio_if;



#endif
