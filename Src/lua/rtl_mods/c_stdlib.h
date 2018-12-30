
#ifndef __C_STDLIB_H
#define __C_STDLIB_H


//RAND_MAX
#ifndef RAND_MAX
#define RAND_MAX C_RAND_MAX
#define C_RAND_MAX 0x7fff
#endif


//EXIT_SUCCESS, EXIT_FAILURE
#ifndef EXIT_SUCCESS
#define EXIT_SUCCESS C_EXIT_SUCCESS
#define C_EXIT_SUCCESS 0
#endif
#ifndef EXIT_FAILURE
#define EXIT_FAILURE C_EXIT_FAILURE
#define C_EXIT_FAILURE 1
#endif


//size_t
typedef unsigned int size_t;

//NULL
#ifndef NULL
#define NULL ((void*)0)
#endif



//free
#define free c_free
void c_free( void* ptr );


//realloc
#define realloc c_realloc
void* c_realloc( void* ptr, size_t size );


//abort
#define abort c_abort
void c_abort( void );


//rand
#define rand c_rand
int c_rand( void );


//srand
#define srand c_srand
void c_srand( unsigned int seed );


//strtod
#define strtod c_strtod
double c_strtod( const char* str, char** endptr );


//system
#define system c_system
int c_system( const char* command );


//getenv
#define getenv c_getenv
char* c_getenv( const char* name );


//exit
#define exit c_exit
void c_exit( int status );


#endif
