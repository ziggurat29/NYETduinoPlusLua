//XXX stub implementation


#include <stdlib.h>


//free
void c_free( void* ptr )
{
	return;	//XXX
}


//realloc
void* c_realloc( void* ptr, size_t size )
{
	return NULL;	//XXX
}


//abort
void c_abort( void )
{
	return;	//XXX
}


//rand
int c_rand( void )
{
	return RAND_MAX;	//XXX
}


//srand
void c_srand( unsigned int seed )
{
	return;	//XXX
}


//strtod
double c_strtod( const char* str, char** endptr )
{
	return 0.0;	//XXX
}


//system
int c_system( const char* command )
{
	return EXIT_FAILURE;	//XXX
}


//getenv
char* c_getenv( const char* name )
{
	return NULL;	//XXX
}


//exit
void c_exit( int status )
{
	return;	//XXX
}


