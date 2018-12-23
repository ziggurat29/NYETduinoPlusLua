//impl
#include "dev_stdio.h"
#include <errno.h>



//========================================================================
//root namespace device
//this thunks over to another device, usually filesystem


int _root_open_r ( struct _reent* r, const char* path, int flags, int mode, void* pdata );
int _root_close_r ( struct _reent* r, int fd, void* pdata );
_ssize_t _root_write_r ( struct _reent* r, int fd, const void* ptr, size_t len, void* pdata );
_ssize_t _root_read_r ( struct _reent* r, int fd, void* ptr, size_t len, void* pdata );
off_t _root_lseek_r ( struct _reent* r, int fd, off_t off, int whence, void* pdata );
void* _root_opendir_r ( struct _reent* r, const char* name, void* pdata );
struct dm_dirent* _root_readdir_r ( struct _reent* r, void* dir, void* pdata );
int _root_closedir_r ( struct _reent* r, void* dir, void* pdata );
const char* _root_getaddr_r ( struct _reent* r, int fd, void* pdata );
int _root_mkdir_r ( struct _reent* r, const char* pathname, mkdir_mode_t mode, void* pdata );
int _root_unlink_r ( struct _reent* r, const char* fname, void* pdata );
int _root_rmdir_r ( struct _reent* r, const char* fname, void* pdata );
int _root_rename_r ( struct _reent* r, const char* oldname, const char* newname, void* pdata );

const DM_DEVICE g_devRoot_if = 
{
	.p_open_r = _root_open_r,
	.p_close_r = _root_close_r,
	.p_write_r = _root_write_r,
	.p_read_r = _root_read_r,
	.p_lseek_r = _root_lseek_r,
	.p_opendir_r = _root_opendir_r,
	.p_readdir_r = _root_readdir_r,
	.p_closedir_r = _root_closedir_r,
	.p_getaddr_r = _root_getaddr_r,
	.p_mkdir_r = _root_mkdir_r,
	.p_unlink_r = _root_unlink_r,
	.p_rmdir_r = _root_rmdir_r,
	.p_rename_r = _root_rename_r,
};


//XXX fd of device alias


int _root_open_r( struct _reent* r, const char* path, int flags, int mode, void* pdata )
{
	//XXX III
	r->_errno = EACCES;
	return -1;
}
int _root_close_r( struct _reent* r, int fd, void* pdata )
{
	//XXX III
	r->_errno = EACCES;
	return -1;
}
_ssize_t _root_write_r ( struct _reent* r, int fd, const void* ptr, size_t len, void* pdata )
{
	//XXX III
	r->_errno = EACCES;
	return -1;
}
_ssize_t _root_read_r( struct _reent* r, int fd, void* ptr, size_t len, void* pdata )
{
	//XXX III
	r->_errno = EACCES;
	return -1;
}


off_t _root_lseek_r ( struct _reent* r, int fd, off_t off, int whence, void* pdata )
{
	//XXX III
	r->_errno = EACCES;
	return 0;
}
void* _root_opendir_r ( struct _reent* r, const char* name, void* pdata )
{
	//XXX III
	r->_errno = EACCES;
	return NULL;
}
struct dm_dirent* _root_readdir_r ( struct _reent* r, void* dir, void* pdata )
{
	//XXX III
	r->_errno = EACCES;
	return NULL;
}
int _root_closedir_r ( struct _reent* r, void* dir, void* pdata )
{
	//XXX III
	r->_errno = EACCES;
	return -1;
}
const char* _root_getaddr_r ( struct _reent* r, int fd, void* pdata )
{
	//XXX III
	r->_errno = EACCES;
	return NULL;
}
int _root_mkdir_r ( struct _reent* r, const char* pathname, mkdir_mode_t mode, void* pdata )
{
	//XXX III
	r->_errno = EACCES;
	return -1;
}
int _root_unlink_r ( struct _reent* r, const char* fname, void* pdata )
{
	//XXX III
	r->_errno = EACCES;
	return -1;
}
int _root_rmdir_r ( struct _reent* r, const char* fname, void* pdata )
{
	//XXX III
	r->_errno = EACCES;
	return -1;
}
int _root_rename_r ( struct _reent* r, const char* oldname, const char* newname, void* pdata )
{
	//XXX III
	r->_errno = EACCES;
	return -1;
}
