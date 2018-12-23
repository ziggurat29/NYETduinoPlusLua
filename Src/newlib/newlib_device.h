//This is the collection of devices in the system, and is used for implementing
//the newlib bottom edge.
//This follows a more-or-less unixian style, and is patterned after eLua's
//realization.
//Significant differences are that I am making this read-only -- i.e. compile-
//time definition of the device tree.
//I may eventually go back to a RAM based device manager, because that allows
//'mounting' devices on-the-fly (which also causes gpio pin assignments to
//change, and various init methods to be called).  However, I am avoiding that
//for now in the interest of simplicity.  Your system's device tree is defined
//in newlib_device.c, where there is a const struct of all the devices.

#ifndef __NEWLIB_DEVICE_H
#define __NEWLIB_DEVICE_H

#include <stdint.h>
#include <reent.h>
#include <unistd.h>
#include <sys/types.h>


// Maximum number of devices in the system
#define DM_MAX_DEVICES        16
#define DM_MAX_DEVICES_BITS   4

// Maximum number of a device name
#define DM_MAX_DEV_NAME       12

// GLOBAL maximum file length (on ALL supported filesystem)
#define DM_MAX_FNAME_LENGTH   30

// Pack/unpack descriptions
// Even if a file descriptor is an 'int', newlib treats it as a short, so we need to stuff
// everything into 16 bits. Actually 15, since negative fd's are actually error indications
#define DM_MAKE_DESC( devid, fd ) ( ( ( devid ) << ( 15 - DM_MAX_DEVICES_BITS ) ) | ( fd ) )
#define DM_GET_DEVID( desc )      ( ( desc ) >> ( 15 - DM_MAX_DEVICES_BITS ) )
#define DM_GET_FD( desc )         ( ( desc ) & ( ( 1 << ( 15 - DM_MAX_DEVICES_BITS ) ) - 1 ) )

// STDIO file number
#define DM_STDIN_NUM              0
#define DM_STDOUT_NUM             1
#define DM_STDERR_NUM             2

// Directory entry flags
#define DM_DIRENT_FLAG_DIR        1

// Our platform independent "dirent" structure (for opendir/readdir)
struct dm_dirent {
	uint32_t fsize;
	const char* fname;
	uint32_t ftime;
	uint8_t flags;
};
typedef struct {
	uint8_t devid;
	void* userdata;
} DM_DIR;

#define DM_DIRENT_IS_DIR( ent )   ( ( ( ent )->flags & DM_DIRENT_FLAG_DIR ) != 0 )

// A device structure with pointers to all the device functions
typedef int mkdir_mode_t;

typedef struct
{
	//newlib required stuff
	int (*p_open_r)( struct _reent* r, const char* path, int flags, int mode, void* pdata );
	int (*p_close_r)( struct _reent* r, int fd, void* pdata );
	_ssize_t (*p_write_r)( struct _reent* r, int fd, const void* ptr, size_t len, void* pdata );
	_ssize_t (*p_read_r)( struct _reent* r, int fd, void* ptr, size_t len, void* pdata );
	off_t (*p_lseek_r)( struct _reent* r, int fd, off_t off, int whence, void* pdata );
//
	void* (*p_opendir_r)( struct _reent* r, const char* name, void* pdata );
	struct dm_dirent* (*p_readdir_r)( struct _reent* r, void* dir, void* pdata );
	int (*p_closedir_r)( struct _reent* r, void* dir, void* pdata );
//
	const char* (*p_getaddr_r)( struct _reent* r, int fd, void* pdata );
//
	int (*p_mkdir_r)( struct _reent* r, const char* pathname, mkdir_mode_t mode, void* pdata );
	int (*p_unlink_r)( struct _reent* r, const char* fname, void* pdata );
	int (*p_rmdir_r)( struct _reent* r, const char* fname, void* pdata );
	int (*p_rename_r)( struct _reent* r, const char* oldname, const char* newname, void* pdata );
//ioctl?
//mount/umount?
} DM_DEVICE;



// DM specific functions (uniform over all the installed filesystems)
DM_DIR* dm_opendir( const char* dirname );
struct dm_dirent* dm_readdir( DM_DIR* d );
int dm_closedir( DM_DIR* d );
const char* dm_getaddr( int fd );



//there is a collection of device instance structs for each registered device
typedef struct
{
	const char* name;		//the device name
	void* pdata;			//device-specific instance data, if any
	const DM_DEVICE* pdev;	//the v-table to the device implementation
} DM_INSTANCE_DATA;



//find device data index, or <0
int find_dm_index( const char* name, char** pactname );
const DM_INSTANCE_DATA* get_dm_entry ( int nIdx );


#endif
