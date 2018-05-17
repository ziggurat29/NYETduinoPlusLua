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
//XXX III do I need to do something interesting?  with the freertos?
	while(1);
}



//execve
//Transfer control to a new process.
int execve(char* name, char** argv, char** env)
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
int _fork_r(void* reent)
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
int _wait_r(void* reent, int* status)
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
int _open_r(void* reent, const char* file, int flags, int mode)
{
//XXX III
	//(struct _reent*)reent
	return 0;
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
int _close_r(void* reent, int fd)
{
//XXX III
	//(struct _reent*)reent
	return 0;
}



//read
//Read from a file.
int read(int file, char* ptr, int len)
{
	return NEEDS_IMPL();
}
//_read_r
//A reentrant version of read. It takes a pointer to the global data block,
//which holds errno.
long _read_r(void* reent, int fd, void* buf, size_t cnt)
{
//XXX III
	//(struct _reent*)reent
	return 0;
}



//write
//Write to a file. libc subroutines will use this system routine for output to
//all files, including stdout—so if you need to generate any output, for
//example to a serial port for debugging, you should make your minimal write
//capable of doing this.
int write(int file, char* ptr, int len)
{
	return NEEDS_IMPL();
}
//_write_r
//A reentrant version of write. It takes a pointer to the global data block,
//which holds errno.
long _write_r(void* reent, int fd, const void* buf, size_t cnt)
{
//XXX III
	//(struct _reent*)reent
	return 0;
}



//lseek
//Set position in a file.
int lseek(int file, int ptr, int dir)
{
	return NEEDS_IMPL();
}
//_lseek_r
//A reentrant version of lseek. It takes a pointer to the global data block,
//which holds errno.
off_t _lseek_r(void* reent, int fd, off_t pos, int whence)
{
//XXX III
	//(struct _reent*)reent
	return 0;
}



//stat
//Status of a file (by name).
int stat(const char* file, struct stat* st)
{
	return NEEDS_IMPL();
}
//_stat_r
//A reentrant version of stat. It takes a pointer to the global data block,
//which holds errno.
int _stat_r(void* reent, const char* file, struct stat* pstat)
{
	return NEEDS_IMPL();
}



//fstat
//Status of an open file.
int fstat(int file, struct stat* st)
{
	return NEEDS_IMPL();
}
//_fstat_r
//A reentrant version of fstat. It takes a pointer to the global data block,
//which holds errno.
int _fstat_r(void* reent, int fd, struct stat* pstat)
{
//XXX III
	//(struct _reent*)reent
	return 0;
}



//link
//Establish a new name for an existing file.
int link(char* old, char* new)
{
	return NEEDS_IMPL();
}
//_link_r
//A reentrant version of link. It takes a pointer to the global data block,
//which holds errno.
int _link_r(void* reent, const char* old, const char* new)
{
//XXX III
	//(struct _reent*)reent
	return 0;
}



//unlink
//Remove a file’s directory entry.
int unlink(char* name)
{
	return NEEDS_IMPL();
}
//_unlink_r
//A reentrant version of unlink. It takes a pointer to the global data block,
//which holds errno.
int _unlink_r(void* reent, const char* file)
{
//XXX III
	//(struct _reent*)reent
	return 0;
}




//====================================================
//malloc stuff


//sbrk
//Increase program data space. As malloc and related functions depend on this,
//it is useful to have a working implementation.
caddr_t sbrk(int incr)
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
char* _sbrk_r(void* reent, size_t incr)
{
	//DO NOT IMPLEMENT!  If you get linker warnings, then that means that the
	//newlib malloc() implementation is being linked, and we are meant to
	//replace that with our own heap manager.  If you get linker warnings, then
	//that replacement has /NOT/ been done correctly.  The magic is done via
	//linker switch '--wrap', the details of which can be found in the 'main'
	//module.  Long story short; you must 'wrap' malloc, free, realloc, etc.
	return (char*)NEEDS_IMPL();
}


