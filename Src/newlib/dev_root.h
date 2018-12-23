//Definitions for the root namespace pseudo 'device';
//The stuff here interfaces our native implementation with the device manager
//we created for gluing onto newlib.
#ifndef __DEV_ROOT_H
#define __DEV_ROOT_H

#include "newlib_device.h"


//this thunks over to whatever it aliases to (usually a filesystem device)
extern const DM_DEVICE g_devRoot_if;



#endif
