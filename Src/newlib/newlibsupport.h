//Newlib is intended to have a bottom layer 'syscalls' and 'stubs'
//implementation in order to realize things like stdio.h functions in libc on
//your particular platform.
//For some reason, the newlib-nano that comes with STM32Workbench already has
//this layer implemented (maybe it is a do-nothing implementation, or 'weak').
//In any event, we don't have the actual source the vendor used, so we don't
//really know for certain.  (Debugging into it looks slightly different from
//the official newlib source, but maybe I got confused).
//Also, I suppose it may be my personal preference, but I'd rather they 
//have put no implementation in (or at least have the default 
//implementation in a physically separate lib), because I like link errors --
//then it is quite clear what what I need to implement myself! As it is now,
//I have to study the map file for stray traces of the newlib stubs that
//get hoisted in incrementally as I build the code. 
//It's possible the builder was unware of the 
//./configure --disable-newlib-supplied-syscalls
//option.
//There is this, which helps somewhat:
//https://sourceware.org/newlib/libc.html#Syscalls
//except of course that those function signatures do not match the actual
//declarations in newlib.  I'd really have appreciated there simply being a
//header declaring all the methods and objects required.
#ifndef __NEWLIBSUPPORT_H
#define __NEWLIBSUPPORT_H

#include <stddef.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <reent.h>
#include <unistd.h>



//environ
//A pointer to a list of environment variables and their values.
extern char** environ;



//_exit
//Exit a program without cleaning up files. If your system doesn’t provide
//this, it is best to avoid linking with subroutines that require it
//(exit, system).
void exit (int status);

//close
//Close a file.
int close(int file);

//execve
//Transfer control to a new process.
int execve(const char *__path, char * const __argv[], char * const __envp[]);

//fork
//Create a new process.
int fork(void);

//fstat
//Status of an open file.
int fstat(int file, struct stat* st);

//getpid
//Process-ID; this is sometimes used to generate strings unlikely to conflict
//with other processes.
int getpid(void);

//isatty
//Query whether output stream is a terminal.
int isatty(int file);

//kill
//Send a signal.
int kill(int pid, int sig);

//link
//Establish a new name for an existing file.
int link(const char *__path1, const char *__path2);

//lseek
//Set position in a file.
off_t lseek(int __fildes, off_t __offset, int __whence);

//open
//Open a file.
int open(const char* name, int flags, int mode);

//read
//Read from a file.
int read(int __fd, void *__buf, size_t __nbyte);

//sbrk
//Increase program data space. As malloc and related functions depend on this,
//it is useful to have a working implementation.
void* sbrk(ptrdiff_t __incr);

//stat
//Status of a file (by name).
int stat(const char* file, struct stat* st);

//times
//Timing information for current process.
clock_t times(struct tms* buf);

//unlink
//Remove a file’s directory entry.
int unlink(const char *__path);

//wait
//Wait for a child process.
int wait(int* status);

//write
//Write to a file. libc subroutines will use this system routine for output to
//all files, including stdout—so if you need to generate any output, for
//example to a serial port for debugging, you should make your minimal write
//capable of doing this.
int write(int __fd, const void *__buf, size_t __nbyte);

//12.2 Reentrant covers for OS subroutines
//Since the system subroutines are used by other library routines that require
//reentrancy, libc.a provides cover routines (for example, the reentrant
//version of fork is _fork_r). These cover routines are consistent with the
//other reentrant subroutines in this library, and achieve reentrancy by using
//a reserved global data block (see Reentrancy).

//_open_r
//A reentrant version of open. It takes a pointer to the global data block,
//which holds errno.
int _open_r(struct _reent* reent, const char* file, int flags, int mode);

//_close_r
//A reentrant version of close. It takes a pointer to the global data block,
//which holds errno.
int _close_r(struct _reent* reent, int fd);

//_lseek_r
//A reentrant version of lseek. It takes a pointer to the global data block,
//which holds errno.
off_t _lseek_r(struct _reent* reent, int fd, off_t pos, int whence);

//_read_r
//A reentrant version of read. It takes a pointer to the global data block,
//which holds errno.
_ssize_t _read_r(struct _reent* reent, int fd, void* buf, size_t cnt);

//_write_r
//A reentrant version of write. It takes a pointer to the global data block,
//which holds errno.
_ssize_t _write_r(struct _reent* reent, int fd, const void* buf, size_t cnt);

//_fork_r
//A reentrant version of fork. It takes a pointer to the global data block,
//which holds errno.
int _fork_r(struct _reent* reent);

//_wait_r
//A reentrant version of wait. It takes a pointer to the global data block,
//which holds errno.
int _wait_r(struct _reent* reent, int* status);

//_stat_r
//A reentrant version of stat. It takes a pointer to the global data block,
//which holds errno.
int _stat_r(struct _reent* reent, const char* file, struct stat* pstat);

//_fstat_r
//A reentrant version of fstat. It takes a pointer to the global data block,
//which holds errno.
int _fstat_r(struct _reent* reent, int fd, struct stat* pstat);

//_link_r
//A reentrant version of link. It takes a pointer to the global data block,
//which holds errno.
int _link_r(struct _reent* reent, const char* __path1, const char* __path2);

//_unlink_r
//A reentrant version of unlink. It takes a pointer to the global data block,
//which holds errno.
int _unlink_r(struct _reent* reent, const char* file);

//_sbrk_r
//A reentrant version of sbrk. It takes a pointer to the global data block,
//which holds errno.
void* _sbrk_r(struct _reent* reent, ptrdiff_t incr);


#endif
