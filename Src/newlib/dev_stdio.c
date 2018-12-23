//impl
#include "dev_stdio.h"
#include <errno.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>



//========================================================================
//stdio device
//this thunks over to another device and exists solely for supplying
//stdin, stdout, stderr.  Those file descriptors are well-known as 0, 1, and 2.
//To preserve those numerical assignments, sub support redirection, it is
//necessary for there to be a device that will have a device index in the
//0-th position (based on carnal details of how file descriptors are concocted;
//it's not an array as with conventional systems -- it's a bitfield.


int _stdio_open_r( struct _reent* r, const char* path, int flags, int mode, void* pdata );
int _stdio_close_r( struct _reent* r, int fd, void* pdata );
_ssize_t _stdio_write_r ( struct _reent* r, int fd, const void* ptr, size_t len, void* pdata );
_ssize_t _stdio_read_r( struct _reent* r, int fd, void* ptr, size_t len, void* pdata );

const DM_DEVICE g_devStdio_if = 
{
	.p_open_r = _stdio_open_r,
	.p_close_r = _stdio_close_r,
	.p_write_r = _stdio_write_r,
	.p_read_r = _stdio_read_r,
	.p_lseek_r = NULL,
	.p_opendir_r = NULL,	//could for enum of redirections
	.p_readdir_r = NULL,	//could for enum of redirections
	.p_closedir_r = NULL,	//could for enum of redirections
	.p_getaddr_r = NULL,
	.p_mkdir_r = NULL,
	.p_unlink_r = NULL,
	.p_rmdir_r = NULL,
	.p_rename_r = NULL,
};


struct DevStdioInst
{
	//fd array for stdin, out, err
	int _fdStdInOutErr[3];
};

struct DevStdioInst g_devStdio_inst = { { -1, -1, -1 } };	//-1 == not redirected



int _stdio_open_r( struct _reent* r, const char* path, int flags, int mode, void* pdata )
{
	//magic; for this device, the fname is the textual 'native' file descriptor
	//to which we are redirecting of targethandle=sourcehandle
	int nTarget;
	int nSource ;
#if 0
	int nCvt = sscanf ( path, "%d=%d", &nTarget, &nSource );
	if ( 2 != nCvt )	//must parse both fields
	{
		r->_errno = EINVAL;	//bad parameter
		return -1;
	}
#else
	const char* pchAt = strchr ( path, '=' );
	if ( NULL == pchAt )
	{
		r->_errno = EINVAL;	//bad parameter
		return -1;
	}
	nTarget = (int) strtol ( path, NULL, 10 );
	nSource = (int) strtol ( pchAt+1, NULL, 10 );
#endif
	if ( nTarget > 2 )	//must be 0, 1, or 2
	{
		r->_errno = EBADF;	//bad file descriptor
		return -1;
	}
//XXX can I make a 'testfd' function to verify file descriptors with their driver?
	if ( -1 != g_devStdio_inst._fdStdInOutErr[nTarget] )	//must not already redirected
	{
		r->_errno = EEXIST;	//exists; already in use
		return -1;
	}
	if ( nTarget == nSource )	//cannot redirect to oneself
	{
		r->_errno = EINVAL;	//bad parameter
		return -1;
	}
	switch ( nTarget )
	{
		case 0:	//stdin must be to a non-'std' device
		case 1:	//stdout must be to a non-'std' device
		if ( nSource < 3 )
		{
			r->_errno = EINVAL;	//bad parameter
			return -1;
		}
		break;
		case 2:	//stderr /can/ be to stdout
		if ( 0 == nSource )	//(failure case of 2 is already handled earlier)
		{
			r->_errno = EINVAL;	//bad parameter
			return -1;
		}
		break;
	}
	
	//OK, simply stow the handle to which we will redirect our I/O
	g_devStdio_inst._fdStdInOutErr[nTarget] = nSource;	//redirect
	r->_errno = 0;	//success
	return nTarget;	//success
}


int _stdio_close_r( struct _reent* r, int fd, void* pdata )
{
	if ( fd > 2 || -1 == g_devStdio_inst._fdStdInOutErr[fd] )
	{
		r->_errno = EBADF;	//bad file descriptor
		return -1;
	}

	//'closing' for us simply means ending the redirection
	g_devStdio_inst._fdStdInOutErr[fd] = -1;
	r->_errno = 0;	//success
	return 0;
}


_ssize_t _stdio_write_r ( struct _reent* r, int fd, const void* ptr, size_t len, void* pdata )
{
	if ( fd > 2 || -1 == g_devStdio_inst._fdStdInOutErr[fd] )
	{
		r->_errno = EBADF;	//bad file descriptor
		return -1;
	}
	
	//OK, now forward to actual device
	int nDevIdx = DM_GET_DEVID( g_devStdio_inst._fdStdInOutErr[fd] );
	int nDevFd = DM_GET_FD( g_devStdio_inst._fdStdInOutErr[fd] );
	const DM_INSTANCE_DATA* pinst = get_dm_entry ( nDevIdx );
	if ( NULL == pinst )
	{
		r->_errno = ENODEV;
		return -1; 
	}
	if( NULL == pinst->pdev->p_write_r )
	{
		r->_errno = ENOSYS;
		return -1;
	}
	
	//forward to device
	return pinst->pdev->p_write_r ( r, nDevFd, ptr, len, pinst->pdata );
}


_ssize_t _stdio_read_r( struct _reent* r, int fd, void* ptr, size_t len, void* pdata )
{
	if ( fd > 2 || -1 == g_devStdio_inst._fdStdInOutErr[fd] )
	{
		r->_errno = EBADF;	//bad file descriptor
		return -1;
	}
	
	//decompose FD into dev idx and native fd and get dev instance
	int nDevIdx = DM_GET_DEVID( g_devStdio_inst._fdStdInOutErr[fd] );
	int nDevFd = DM_GET_FD( g_devStdio_inst._fdStdInOutErr[fd] );
	const DM_INSTANCE_DATA* pinst = get_dm_entry ( nDevIdx );
	if ( NULL == pinst )
	{
		r->_errno = ENODEV;
		return -1; 
	}
	if( NULL == pinst->pdev->p_read_r )
	{
		r->_errno = ENOSYS;
		return -1;
	}
	
	//forward to device
	return pinst->pdev->p_read_r ( r, nDevFd, ptr, len, pinst->pdata );
}


