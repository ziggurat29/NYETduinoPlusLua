//impl
#include "dev_serial.h"
#include <errno.h>

#include "../serial_devices.h"	//platform-specific interfaces


//XXX could use the device-specific instance structure to factor this
//implementation; especially since we already went to the trouble to
//do so in serial_devices.h with the IOStreamIF implementation


//========================================================================
//USART6 device
//this can only be opened once


int _USART6_open_r( struct _reent* r, const char* path, int flags, int mode, void* pdata );
int _USART6_close_r( struct _reent* r, int fd, void* pdata );
_ssize_t _USART6_write_r ( struct _reent* r, int fd, const void* ptr, size_t len, void* pdata );
_ssize_t _USART6_read_r( struct _reent* r, int fd, void* ptr, size_t len, void* pdata );

const DM_DEVICE g_devUSART6_if = 
{
	.p_open_r = _USART6_open_r,
	.p_close_r = _USART6_close_r,
	.p_write_r = _USART6_write_r,
	.p_read_r = _USART6_read_r,
	.p_lseek_r = NULL,
	.p_opendir_r = NULL,
	.p_readdir_r = NULL,
	.p_closedir_r = NULL,
	.p_getaddr_r = NULL,
	.p_mkdir_r = NULL,
	.p_unlink_r = NULL,
	.p_rmdir_r = NULL,
	.p_rename_r = NULL,
};



struct DevUsart6Inst
{
	uint8_t	_bOpened;
};

struct DevUsart6Inst g_devUsart6_inst = { 0 };



int _USART6_open_r( struct _reent* r, const char* path, int flags, int mode, void* pdata )
{
	if ( g_devUsart6_inst._bOpened )	//only one at a time
	{
		r->_errno = EACCES;
		return -1;
	}
	
	g_devUsart6_inst._bOpened = 1;
	r->_errno = 0;
	return 0;
}


int _USART6_close_r( struct _reent* r, int fd, void* pdata )
{
	if ( ! g_devUsart6_inst._bOpened )	//must be opened
	{
		r->_errno = EBADF;
		return -1;
	}
	
	g_devUsart6_inst._bOpened = 0;
	r->_errno = 0;
	return 0;
}



_ssize_t _USART6_write_r ( struct _reent* r, int fd, const void* ptr, size_t len, void* pdata )
{
	if ( ! g_devUsart6_inst._bOpened )	//must be opened
	{
		r->_errno = EBADF;
		return -1;
	}
	
	size_t nRet = g_iosUART6._transmit ( &g_iosUART6, ptr, len );
	r->_errno = 0;
	return (_ssize_t)nRet;
}



_ssize_t _USART6_read_r( struct _reent* r, int fd, void* ptr, size_t len, void* pdata )
{
	if ( ! g_devUsart6_inst._bOpened )	//must be opened
	{
		r->_errno = EBADF;
		return -1;
	}
	
	size_t nRet = g_iosUART6._receive ( &g_iosUART6, ptr, len );
	r->_errno = 0;
	return (_ssize_t)nRet;
}



//========================================================================
//USB CDC device
//this can only be opened once


int _CDC_open_r( struct _reent* r, const char* path, int flags, int mode, void* pdata );
int _CDC_close_r( struct _reent* r, int fd, void* pdata );
_ssize_t _CDC_write_r ( struct _reent* r, int fd, const void* ptr, size_t len, void* pdata );
_ssize_t _CDC_read_r( struct _reent* r, int fd, void* ptr, size_t len, void* pdata );

const DM_DEVICE g_devCDC_if = 
{
	.p_open_r = _CDC_open_r,
	.p_close_r = _CDC_close_r,
	.p_write_r = _CDC_write_r,
	.p_read_r = _CDC_read_r,
	.p_lseek_r = NULL,
	.p_opendir_r = NULL,
	.p_readdir_r = NULL,
	.p_closedir_r = NULL,
	.p_getaddr_r = NULL,
	.p_mkdir_r = NULL,
	.p_unlink_r = NULL,
	.p_rmdir_r = NULL,
	.p_rename_r = NULL,
};



struct DevCDC6Inst
{
	uint8_t	_bOpened;
};

struct DevCDC6Inst g_devCDC6_inst = { 0 };



int _CDC_open_r( struct _reent* r, const char* path, int flags, int mode, void* pdata )
{
	if ( g_devCDC6_inst._bOpened )	//only one at a time
	{
		r->_errno = EACCES;
		return -1;
	}
	
	g_devCDC6_inst._bOpened = 1;
	r->_errno = 0;
	return 0;
}


int _CDC_close_r( struct _reent* r, int fd, void* pdata )
{
	if ( ! g_devCDC6_inst._bOpened )	//must be opened
	{
		r->_errno = EBADF;
		return -1;
	}
	
	g_devCDC6_inst._bOpened = 0;
	r->_errno = 0;
	return 0;
}


_ssize_t _CDC_write_r ( struct _reent* r, int fd, const void* ptr, size_t len, void* pdata )
{
	if ( ! g_devCDC6_inst._bOpened )	//must be opened
	{
		r->_errno = EBADF;
		return -1;
	}
	
	size_t nRet = g_iosCDC._transmit ( &g_iosCDC, ptr, len );
	r->_errno = 0;
	return (_ssize_t)nRet;
}


_ssize_t _CDC_read_r( struct _reent* r, int fd, void* ptr, size_t len, void* pdata )
{
	if ( ! g_devCDC6_inst._bOpened )	//must be opened
	{
		r->_errno = EBADF;
		return -1;
	}
	
	size_t nRet = g_iosCDC._receive ( &g_iosCDC, ptr, len );
	r->_errno = 0;
	return (_ssize_t)nRet;
}



