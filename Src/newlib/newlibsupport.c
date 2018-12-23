//Newlib is intended to have a bottom layer 'syscalls' and 'stubs'
//implementation in order to realize things like stdio.h functions in libc on
//your particular platform.

#include "newlibsupport.h"
#include "newlib_device.h"

#include <errno.h>
#undef errno
extern int errno;	//hmm...


//As I have already complained; the newlib-nano that comes with STM32Workbench
//already has an internal implementation of the low-level syscalls/stubs
//methods.  I find this very inconvenient, because I have to do a lot of
//research with the map file to figure out which of those gets hoisted into the
//link to know what I need to implement.  I would normally just leave all these
//methods unimplemented, and then the linker would tell you, but since the
//'defaults' are there, the linker is of no help.  So, I try to deliberately
//break the link by overriding all the stubs, and having the override
//implementation call this unimplemented function.  Now the linker will point
//out which methods need to be implemented for you.  You really only have to
//provide implementation for the needed ones, and leave the others alone.
extern int NEEDS_IMPL ( void );



//====================================================
//system stuff



//For a minimal environment, this empty list is adequate (pity it wastes
//RAM; you might try commenting it out and see if you can get away with
//it):
//char*__env[1] = { 0 };
//char** environ = __env;
static char const * const __env[] = { "", NULL };
char** environ = (char**)__env;



//_exit
//Exit a program without cleaning up files. If your system doesn’t provide
//this, it is best to avoid linking with subroutines that require it
//(exit, system).
void exit (int status)
{
//XXX III do I need to do something interesting?  with the freertos?
	while(1);
}



//execve
//Transfer control to a new process.
int execve(const char *__path, char * const __argv[], char * const __envp[])
{
	return NEEDS_IMPL();
}


//fork
//Create a new process.
int fork(void)
{
	return NEEDS_IMPL();
}
//_fork_r
//A reentrant version of fork. It takes a pointer to the global data block,
//which holds errno.
int _fork_r(struct _reent* reent)
{
	return NEEDS_IMPL();
}


//wait
//Wait for a child process.
int wait(int* status)
{
	return NEEDS_IMPL();
}
//_wait_r
//A reentrant version of wait. It takes a pointer to the global data block,
//which holds errno.
int _wait_r(struct _reent* reent, int* status)
{
	return NEEDS_IMPL();
}


//getpid
//Process-ID; this is sometimes used to generate strings unlikely to conflict
//with other processes.
int getpid(void)
{
	return NEEDS_IMPL();
}


//kill
//Send a signal.
int kill(int pid, int sig)
{
	return NEEDS_IMPL();
}


//times
//Timing information for current process.
clock_t times(struct tms* buf)
{
	return (clock_t)NEEDS_IMPL();
}



//isatty
//Query whether output stream is a terminal.
int isatty(int file)
{
	return 1;
	//return NEEDS_IMPL();
}



//====================================================
//filesystem stuff



//open
//Open a file.
int open(const char* name, int flags, int mode)
{
	return _open_r( _REENT, name, flags, 0 );
}
//_open_r
//A reentrant version of open. It takes a pointer to the global data block,
//which holds errno.
int _open_r(struct _reent* reent, const char* file, int flags, int mode)
{
	//find device and lop off name, if needed
	char* actname;
	int nDevIdx = find_dm_index( file, &actname );
	const DM_INSTANCE_DATA* pinst = get_dm_entry ( nDevIdx );
	if ( NULL == pinst )
	{
		reent->_errno = ENODEV;
		return -1; 
	}
	if( NULL == pinst->pdev->p_open_r )
	{
		reent->_errno = ENOSYS;
		return -1;
	}
	
	//forward to device
	int nDevFd = pinst->pdev->p_open_r ( reent, actname, flags, mode, pinst->pdata );
	if ( nDevFd < 0 )
	{
		//(errno has already been set)
		return -1;
	}
	
	//concoct our logical file descriptor from the device file descriptor
	int nFd = DM_MAKE_DESC ( nDevIdx, nDevFd );
	
	return nFd;
}



//close
//Close a file.
int close(int fd)
{
	return _close_r( _REENT, fd );
}
//_close_r
//A reentrant version of close. It takes a pointer to the global data block,
//which holds errno.
int _close_r(struct _reent* reent, int fd)
{
	//decompose FD into dev idx and native fd and get dev instance
	int nDevIdx = DM_GET_DEVID( fd );
	int nDevFd = DM_GET_FD( fd );
	const DM_INSTANCE_DATA* pinst = get_dm_entry ( nDevIdx );
	if ( NULL == pinst )
	{
		reent->_errno = ENODEV;
		return -1; 
	}
	if( NULL == pinst->pdev->p_close_r )
	{
		reent->_errno = ENOSYS;
		return -1;
	}
	
	//forward to device
	return pinst->pdev->p_close_r ( reent, nDevFd, pinst->pdata );
}



//read
//Read from a file.
_READ_WRITE_RETURN_TYPE read(int __fd, void *__buf, size_t __nbyte)
{
	return _read_r(_REENT, __fd, __buf, __nbyte);
}
//_read_r
//A reentrant version of read. It takes a pointer to the global data block,
//which holds errno.
_ssize_t _read_r(struct _reent* reent, int fd, void* buf, size_t cnt)
{
	//decompose FD into dev idx and native fd and get dev instance
	int nDevIdx = DM_GET_DEVID( fd );
	int nDevFd = DM_GET_FD( fd );
	const DM_INSTANCE_DATA* pinst = get_dm_entry ( nDevIdx );
	if ( NULL == pinst )
	{
		reent->_errno = ENODEV;
		return -1; 
	}
	if( NULL == pinst->pdev->p_read_r )
	{
		reent->_errno = ENOSYS;
		return -1;
	}
	
	//forward to device
	return pinst->pdev->p_read_r ( reent, nDevFd, buf, cnt, pinst->pdata );
}



//write
//Write to a file. libc subroutines will use this system routine for output to
//all files, including stdout—so if you need to generate any output, for
//example to a serial port for debugging, you should make your minimal write
//capable of doing this.
_READ_WRITE_RETURN_TYPE write(int fd, const void* buf, size_t nbyte)
{
	return _write_r(_REENT, fd, buf, nbyte);
}
//_write_r
//A reentrant version of write. It takes a pointer to the global data block,
//which holds errno.
_ssize_t _write_r(struct _reent* reent, int fd, const void* buf, size_t cnt)
{
	//decompose FD into dev idx and native fd and get dev instance
	int nDevIdx = DM_GET_DEVID( fd );
	int nDevFd = DM_GET_FD( fd );
	const DM_INSTANCE_DATA* pinst = get_dm_entry ( nDevIdx );
	if ( NULL == pinst )
	{
		reent->_errno = ENODEV;
		return -1; 
	}
	if( NULL == pinst->pdev->p_write_r )
	{
		reent->_errno = ENOSYS;
		return -1;
	}
	
	//forward to device
	return pinst->pdev->p_write_r ( reent, nDevFd, buf, cnt, pinst->pdata );
}



//lseek
//Set position in a file.
off_t lseek(int fd, off_t offset, int whence)
{
	return _lseek_r(_REENT, fd, offset, whence);
}
//_lseek_r
//A reentrant version of lseek. It takes a pointer to the global data block,
//which holds errno.
off_t _lseek_r(struct _reent* reent, int fd, off_t offset, int whence)
{
	//decompose FD into dev idx and native fd and get dev instance
	int nDevIdx = DM_GET_DEVID( fd );
	int nDevFd = DM_GET_FD( fd );
	const DM_INSTANCE_DATA* pinst = get_dm_entry ( nDevIdx );
	if ( NULL == pinst )
	{
		reent->_errno = ENODEV;
		return -1; 
	}
	if( NULL == pinst->pdev->p_lseek_r )
	{
		reent->_errno = ENOSYS;
		return -1;
	}
	
	//forward to device
	return pinst->pdev->p_lseek_r ( reent, nDevFd, offset, whence, pinst->pdata );
}



//stat
//Status of a file (by name).
int stat(const char* file, struct stat* pstat)
{
	return _stat_r(_REENT, file, pstat);
}
//_stat_r
//A reentrant version of stat. It takes a pointer to the global data block,
//which holds errno.
int _stat_r(struct _reent* reent, const char* file, struct stat* pstat)
{
	return NEEDS_IMPL();
}



//fstat
//Status of an open file.
int fstat(int fd, struct stat* pstat)
{
	return _fstat_r(_REENT, fd, pstat);
}
//_fstat_r
//A reentrant version of fstat. It takes a pointer to the global data block,
//which holds errno.
int _fstat_r(struct _reent* reent, int fd, struct stat* pstat)
{
	//(not really implemented)
	if( ( fd >= DM_STDIN_NUM ) && ( fd <= DM_STDERR_NUM ) )
	{
		pstat->st_mode = S_IFCHR;
		return 0;
	}
	reent->_errno = ENOSYS;
	return -1;
}



//link
//Establish a new name for an existing file.
int link(const char *path1, const char *path2)
{
	return _link_r(_REENT, path1, path2);
}
//_link_r
//A reentrant version of link. It takes a pointer to the global data block,
//which holds errno.
int _link_r(struct _reent* reent, const char* path1, const char* path2)
{
//XXX III
	//(struct _reent*)reent
	return 0;
}



//unlink
//Remove a file’s directory entry.
int unlink(const char* file)
{
	return _unlink_r(_REENT, file);
}
//_unlink_r
//A reentrant version of unlink. It takes a pointer to the global data block,
//which holds errno.
int _unlink_r(struct _reent* reent, const char* file)
{
	//find device and lop off name, if needed
	char* actname;
	int nDevIdx = find_dm_index( file, &actname );
	const DM_INSTANCE_DATA* pinst = get_dm_entry ( nDevIdx );
	if ( NULL == pinst )
	{
		reent->_errno = ENODEV;
		return -1; 
	}
	if( NULL == pinst->pdev->p_unlink_r )
	{
		reent->_errno = ENOSYS;
		return -1;
	}
	
	//forward to device
	return pinst->pdev->p_unlink_r ( reent, actname, pinst->pdata );
}




//====================================================
//malloc stuff


//sbrk
//Increase program data space. As malloc and related functions depend on this,
//it is useful to have a working implementation.
void* sbrk(ptrdiff_t incr)
{
	//DO NOT IMPLEMENT!  If you get linker warnings, then that means that the
	//newlib malloc() implementation is being linked, and we are meant to
	//replace that with our own heap manager.  If you get linker warnings, then
	//that replacement has /NOT/ been done correctly.  The magic is done via
	//linker switch '--wrap', the details of which can be found in the 'main'
	//module.  Long story short; you must 'wrap' malloc, free, realloc, etc.
	return (caddr_t)NEEDS_IMPL();
}



//_sbrk_r
//A reentrant version of sbrk. It takes a pointer to the global data block,
//which holds errno.
void* _sbrk_r(struct _reent* reent, ptrdiff_t incr)
{
	//DO NOT IMPLEMENT!  If you get linker warnings, then that means that the
	//newlib malloc() implementation is being linked, and we are meant to
	//replace that with our own heap manager.  If you get linker warnings, then
	//that replacement has /NOT/ been done correctly.  The magic is done via
	//linker switch '--wrap', the details of which can be found in the 'main'
	//module.  Long story short; you must 'wrap' malloc, free, realloc, etc.
	return (char*)NEEDS_IMPL();
}


