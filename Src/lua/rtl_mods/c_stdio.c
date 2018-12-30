//XXX stub implementation


#include <stdio.h>
#include <stdarg.h>


typedef struct c_iobuf
{
	void* _Placeholder;
} C_FILE;


C_FILE* c_acrt_iob_func( unsigned n )
{
	return NULL;	//XXX
}


//fopen
C_FILE* c_fopen(
	char const* _FileName,
	char const* _Mode
)
{
	return NULL;	//XXX
}


//feof
int c_feof(
	C_FILE* _Stream
)
{
	return 0;	//XXX
}


//fread
size_t c_fread(
	void* _Buffer,
	size_t _ElementSize,
	size_t _ElementCount,
	C_FILE* _Stream
)
{
	return 0;	//XXX
}


//getc
int c_getc(
	C_FILE* _Stream
)
{
	return -1;	//XXX
}


//fseek
int c_fseek(
	C_FILE* _Stream,
	long _Offset,
	int _Origin
)
{
	return -1;	//XXX
}


//ftell
long c_ftell(
	C_FILE* _Stream
)
{
	return -1;	//XXX
}


//setvbuf
int c_setvbuf(
	C_FILE* _Stream,
	char* _Buffer,
	int _Mode,
	size_t _Size
)
{
	return -1;	//XXX
}


//sprintf
int c_sprintf(
	char* const _Buffer,
	char const* const _Format,
	... )
{
	va_list va;
	va_start( va, _Format );
	int nRet = vsprintf( _Buffer, _Format, va );
	va_end( va );
	return nRet;
}


//ungetc
int c_ungetc(
	int   _Character,
	C_FILE* _Stream
)
{
	return -1;	//XXX
}


//clearerr
void c_clearerr(
	C_FILE* _Stream
)
{
}


//freopen
C_FILE* c_freopen(
	char const* _FileName,
	char const* _Mode,
	C_FILE*       _Stream
)
{
	return NULL;	//XXX
}


//ferror
int c_ferror(
	C_FILE* _Stream
)
{
	return 1;	//XXX
}


//fclose
int c_fclose(
	C_FILE* _Stream
)
{
	return EOF;	//XXX
}


//fprintf
int c_fprintf(
	C_FILE* const _Stream,
	char const* const _Format,
	... )
{
	return -1;	//XXX
}


//printf
int c_printf(
	char const* const _Format,
	... )
{
	return -1;	//XXX
}


//fflush
int c_fflush(
	C_FILE* _Stream
)
{
	return -1;	//XXX
}


//fwrite
size_t c_fwrite(
	void const* _Buffer,
	size_t _ElementSize,
	size_t _ElementCount,
	C_FILE* _Stream
)
{
	return 0;	//XXX
}


//fgets
char* c_fgets(
	char* _Buffer,
	int _MaxCount,
	C_FILE* _Stream
)
{
	return NULL;	//XXX
}


//fputs
int c_fputs(
	char const* _Buffer,
	FILE* _Stream
)
{
	return EOF;	//XXX
}



//========================================================================


//popen
C_FILE* c_popen(
	char const* _Command,
	char const* _Mode
)
{
	return NULL;	//XXX
}


//pclose
int c_pclose(
	C_FILE* _Stream
)
{
	return 0;	//XXX
}


//tmpfile
C_FILE* c_tmpfile( void )
{
	return NULL;	//XXX
}


//remove
int c_remove(
	char const* _FileName
)
{
	return -1;	//XXX
}

//rename
int c_rename(
	char const* _OldFileName,
	char const* _NewFileName
)
{
	return -1;	//XXX
}

//tmpnam
char* c_tmpnam( char* s )
{
	return NULL;	//XXX
}


//fileno
int c_fileno( C_FILE *stream )
{
	return -1;
}

//isatty
int c_isatty(
	int _FileHandle
)
{
	return 0;	//XXX
}


