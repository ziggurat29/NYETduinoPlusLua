//impl
#include "newlib_device.h"

#include <string.h>

//include all the devices that we are going to define below
#include "dev_stdio.h"
#include "dev_serial.h"

#include "dev_root.h"


#ifndef COUNTOF
#define COUNTOF(arr) (sizeof(arr)/sizeof((arr)[0]))
#endif



const DM_INSTANCE_DATA g_adid[] =
{
	//the first device /must/ be the stdio device, so that the well-known
	//descriptors 0,1,2 will be assigned correctly.  This is a bit of
	//haquery exploiting carnal details of how file descriptors are
	//concocted.
	{
		.name = "/std",
		.pdata = NULL,
		.pdev = &g_devStdio_if,
	},
//XXX is /null useful?
	
	//in-between is as you wish.  Just be careful about how names are
	//looked up (by prefix, so most-specific first).
	{	//USART6
		.name = "/ttys0",
		.pdata = NULL,
		.pdev = &g_devUSART6_if,
	},
	{	//USB CDC
		.name = "/ttys4",
		.pdata = NULL,
		.pdev = &g_devCDC_if,
	},
	
	//the last device /must/ be the root device, so that any name that
	//doesn't get trapped above, gets handled by it.  This is mild
	//haquery based on how names are looked up by prefix.
	{
		.name = "/",
		.pdata = NULL,
		.pdev = &g_devRoot_if,
	}
};




int find_dm_index( const char* name, char** pactname )
{
	// Sanity check for name
	if( name == NULL || *name == '\0' || *name != '/' )
		return -1;
	
	const char* preal;
	char tempname[ DM_MAX_DEV_NAME + 1 ];
	// Find device name
	preal = strchr( name + 1, '/' );
	if( preal == NULL )
	{
		// This shortcut allows to register the "/" filesystem and use it like "/file.ext"
		strcpy( tempname, "/" );
		preal = name;
	}
	else
	{
		if( ( preal - name > DM_MAX_DEV_NAME ) || ( preal - name == 1 ) ) // name too short/too long
			return -1;
		memcpy( tempname, name, preal - name );
		tempname[ preal - name ] = '\0';
	}
	
	const DM_INSTANCE_DATA* pinst = NULL;
	size_t nIdx;
	for ( nIdx = 0; nIdx < COUNTOF(g_adid); ++nIdx )
	{
		pinst = &g_adid[nIdx];
		if( !strcasecmp( tempname, pinst->name ) )
			break;
	}
	if ( NULL == pinst )
		return -1;
	
	// Find the actual first char of the name
	++preal;
	if( '\0' == *preal )
		return -1;
	*pactname = (char*)preal;

	//fin
	return (int)nIdx;
}



const DM_INSTANCE_DATA* get_dm_entry ( int nIdx )
{
	if ( nIdx < 0 || nIdx > COUNTOF(g_adid)-1 )
		return NULL;
	return &g_adid[nIdx];
}


