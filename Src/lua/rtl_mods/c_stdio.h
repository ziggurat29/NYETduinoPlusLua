
#ifndef __C_STDIO_H
#define __C_STDIO_H

#include <stddef.h>


//FILE
#ifndef _FILE_DEFINED
#define _FILE_DEFINED
#define FILE C_FILE
typedef struct c_iobuf
{
	void* _Placeholder;
} C_FILE;
#endif


//BUFSIZ
#define BUFSIZ C_BUFFSIZ
#define C_BUFFSIZ 512


//EOF
#define EOF C_EOF
#define C_EOF (-1)


//_IOFBF, _IOLBF, _IONBF
#define _IOFBF C_IOFBF
#define C_IOFBF 0x0000
#define _IOLBF C_IOLBF
#define C_IOLBF 0x0040
#define _IONBF C_IONBF
#define C_IONBF 0x0004


#ifndef SEEK_SET
#define SEEK_SET C_SEEK_SET
#define C_SEEK_SET 0
#define SEEK_CUR C_SEEK_CUR
#define C_SEEK_CUR 1
#define SEEK_END C_SEEK_END
#define C_SEEK_END 2
#endif



//stdin
//stdout
//stderr
C_FILE* c_acrt_iob_func( unsigned );
#define stdin c_stdin
#define c_stdin  (c_acrt_iob_func(0))
#define stdout c_stdout
#define c_stdout (c_acrt_iob_func(1))
#define stderr c_stderr
#define c_stderr (c_acrt_iob_func(2))



//fopen
#define fopen c_fopen
C_FILE* c_fopen(
	char const* _FileName,
	char const* _Mode
);


//feof
#define feof c_feof
int c_feof(
	C_FILE* _Stream
);


//fread
#define fread c_fread
size_t c_fread(
	void* _Buffer,
	size_t _ElementSize,
	size_t _ElementCount,
	C_FILE* _Stream
);


//getc
#define getc c_getc
int c_getc(
	C_FILE* _Stream
);


//fseek
//XXX
#define _fseeki64 c_fseek
#define fseek c_fseek
int c_fseek(
	C_FILE* _Stream,
	long _Offset,
	int _Origin
);


//ftell
//XXX
#define _ftelli64 c_ftell
#define ftell c_ftell
long c_ftell(
	C_FILE* _Stream
);


//setvbuf
#define setvbuf c_setvbuf
int c_setvbuf(
	C_FILE* _Stream,
	char* _Buffer,
	int _Mode,
	size_t _Size
);


//sprintf
#define sprintf c_sprintf
int c_sprintf(
	char* const _Buffer,
	char const* const _Format,
	... );


//ungetc
#define ungetc c_ungetc
int c_ungetc(
	int   _Character,
	C_FILE* _Stream
);


//clearerr
#define clearerr c_clearerr
void c_clearerr(
	C_FILE* _Stream
);


//freopen
#define freopen c_freopen
C_FILE* c_freopen(
	char const* _FileName,
	char const* _Mode,
	C_FILE*       _Stream
);


//ferror
#define ferror c_ferror
int c_ferror(
	C_FILE* _Stream
);


//fclose
#define fclose c_fclose
int c_fclose(
	C_FILE* _Stream
);


//fprintf
#define fprintf c_fprintf
int c_fprintf(
	C_FILE* const _Stream,
	char const* const _Format,
	... );


//printf
#define printf c_printf
int c_printf(
	char const* const _Format,
	... );


//fflush
#define fflush c_fflush
int c_fflush(
	C_FILE* _Stream
);


//fwrite
#define fwrite c_fwrite
size_t c_fwrite(
	void const* _Buffer,
	size_t _ElementSize,
	size_t _ElementCount,
	C_FILE* _Stream
);


//fgets
#define fgets c_fgets
char* c_fgets(
	char* _Buffer,
	int _MaxCount,
	C_FILE* _Stream
);


//fputs
#define fputs c_fputs
int c_fputs(
	char const* _Buffer,
	FILE* _Stream
);


//popen
//XXX
#define _popen c_popen
C_FILE* c_popen(
	char const* _Command,
	char const* _Mode
);


//pclose
//XXX
#define _pclose c_pclose
int c_pclose(
	C_FILE* _Stream
);


//tmpfile
#define tmpfile c_tmpfile
C_FILE* c_tmpfile( void );


//remove
#define remove c_remove
int c_remove(
	char const* _FileName
);


//rename
#define rename c_rename
int c_rename(
	char const* _OldFileName,
	char const* _NewFileName
);


#define L_tmpnam   C_L_tmpnam
#define C_L_tmpnam   260 // _MAX_PATH

//tmpnam
#define tmpnam c_tmpnam
char* c_tmpnam( char *s );


//fileno
//XXX
#define _fileno c_fileno
#define fileno c_fileno
int c_fileno( C_FILE *stream );


//isatty
//XXX
#define _isatty c_isatty
#define isatty c_isatty
int c_isatty(
	int _FileHandle
);


#endif
