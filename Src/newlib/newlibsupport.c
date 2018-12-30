//Newlib is intended to have a bottom layer 'syscalls' and 'stubs'
//implementation in order to realize things like stdio.h functions in libc on
//your particular platform.

#include "newlibsupport.h"

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


//_exit
//Exit a program without cleaning up files. If your system doesn’t provide
//this, it is best to avoid linking with subroutines that require it
//(exit, system).
void exit (int status)
{
	NEEDS_IMPL();
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
	//(struct _reent*)reent
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
	//(struct _reent*)reent
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
	return NEEDS_IMPL();
}



//====================================================
//filesystem stuff



//open
//Open a file.
int open(const char* name, int flags, int mode)
{
	return NEEDS_IMPL();
}
//_open_r
//A reentrant version of open. It takes a pointer to the global data block,
//which holds errno.
int _open_r(struct _reent* reent, const char* file, int flags, int mode)
{
	//(struct _reent*)reent
	return NEEDS_IMPL();
}



//close
//Close a file.
int close(int file)
{
	return NEEDS_IMPL();
}
//_close_r
//A reentrant version of close. It takes a pointer to the global data block,
//which holds errno.
int _close_r(struct _reent* reent, int fd)
{
	//(struct _reent*)reent
	return NEEDS_IMPL();
}



//read
//Read from a file.
_READ_WRITE_RETURN_TYPE read(int __fd, void *__buf, size_t __nbyte)
{
	return NEEDS_IMPL();
}
//_read_r
//A reentrant version of read. It takes a pointer to the global data block,
//which holds errno.
_ssize_t _read_r(struct _reent* reent, int fd, void* buf, size_t cnt)
{
	//(struct _reent*)reent
	return (long)NEEDS_IMPL();
}



//write
//Write to a file. libc subroutines will use this system routine for output to
//all files, including stdout—so if you need to generate any output, for
//example to a serial port for debugging, you should make your minimal write
//capable of doing this.
_READ_WRITE_RETURN_TYPE write(int __fd, const void *__buf, size_t __nbyte)
{
	return NEEDS_IMPL();
}
//_write_r
//A reentrant version of write. It takes a pointer to the global data block,
//which holds errno.
_ssize_t _write_r(struct _reent* reent, int fd, const void* buf, size_t cnt)
{
	//(struct _reent*)reent
	return (long)NEEDS_IMPL();
}



//lseek
//Set position in a file.
off_t lseek(int __fildes, off_t __offset, int __whence)
{
	return NEEDS_IMPL();
}
//_lseek_r
//A reentrant version of lseek. It takes a pointer to the global data block,
//which holds errno.
off_t _lseek_r(struct _reent* reent, int __fildes, off_t __offset, int __whence)
{
	//(struct _reent*)reent
	return (off_t)NEEDS_IMPL();
}



//stat
//Status of a file (by name).
int stat(const char* file, struct stat* pstat)
{
	return NEEDS_IMPL();
}
//_stat_r
//A reentrant version of stat. It takes a pointer to the global data block,
//which holds errno.
int _stat_r(struct _reent* reent, const char* file, struct stat* pstat)
{
	//(struct _reent*)reent
	return NEEDS_IMPL();
}



//fstat
//Status of an open file.
int fstat(int fd, struct stat* pstat)
{
	return NEEDS_IMPL();
}
//_fstat_r
//A reentrant version of fstat. It takes a pointer to the global data block,
//which holds errno.
int _fstat_r(struct _reent* reent, int fd, struct stat* pstat)
{
	//(struct _reent*)reent
	return NEEDS_IMPL();
}



//link
//Establish a new name for an existing file.
int link(const char *__path1, const char *__path2)
{
	return NEEDS_IMPL();
}
//_link_r
//A reentrant version of link. It takes a pointer to the global data block,
//which holds errno.
int _link_r(struct _reent* reent, const char* __path1, const char* __path2)
{
	//(struct _reent*)reent
	return NEEDS_IMPL();
}



//unlink
//Remove a file’s directory entry.
int unlink(const char* file)
{
	return NEEDS_IMPL();
}
//_unlink_r
//A reentrant version of unlink. It takes a pointer to the global data block,
//which holds errno.
int _unlink_r(struct _reent* reent, const char* file)
{
	//(struct _reent*)reent
	return NEEDS_IMPL();
}




//====================================================
//malloc stuff


//sbrk
//Increase program data space. As malloc and related functions depend on this,
//it is useful to have a working implementation.
void* sbrk(ptrdiff_t incr)
{
	return (caddr_t)NEEDS_IMPL();
}



//_sbrk_r
//A reentrant version of sbrk. It takes a pointer to the global data block,
//which holds errno.
void* _sbrk_r(struct _reent* reent, ptrdiff_t incr)
{
	//(struct _reent*)reent
	return (char*)NEEDS_IMPL();
}


