
//these /probably/ do not need shimming, unless your standard headers'
//implementations happen to internally #include stdio.h or stdlib.h
#define C_HEADER_ASSERT <assert.h>
#define C_HEADER_CTYPE <ctype.h>
#define C_HEADER_ERRNO <errno.h>
#define C_HEADER_FLOAT <float.h>
#define C_HEADER_IO <io.h>
#define C_HEADER_LIMITS <limits.h>
#define C_HEADER_LOCALE <locale.h>
#define C_HEADER_MATH <math.h>
#define C_HEADER_SETJMP <setjmp.h>
#define C_HEADER_SIGNAL <signal.h>
#define C_HEADER_STDARG <stdarg.h>
#define C_HEADER_STDDEF <stddef.h>
#define C_HEADER_STDINT <stdint.h>
#define C_HEADER_STRING <string.h>
#define C_HEADER_SYS_TYPES <sys/types.h>
#define C_HEADER_SYS_WAIT <sys/wait.h>
#define C_HEADER_TIME <time.h>
#define C_HEADER_UNISTD <unistd.h>


//these features are probably more more easily disabled via the luaconf.h, but
//you could shim them if you want.  These headers internally #include stdlib.h,
//which will cause symbol conflicts with the ones we define.

//LUA_DL_DLL:
#define C_HEADER_WINDOWS <windows.h>
//LUA_USE_DLOPEN:
#define C_HEADER_DLFCN <dlfcn.h>
//LUA_USE_READLINE:
#define C_HEADER_READLINE_READLINE <readline/readline.h>
#define C_HEADER_READLINE_HISTORY <readline/history.h>



//these will certainly need to be shimmed for embedded stuff
#if USE_LIBC
//if you want to use libc implementation

#define C_HEADER_STDIO <stdio.h>
#define C_HEADER_STDLIB <stdlib.h>

#else
//if you want to implement your shims for your libc replacements

//these are not complete libc implementations, but rather only just what
//Lua internally requires.

#define C_HEADER_STDIO "rtl_mods\c_stdio.h"
#define C_HEADER_STDLIB "rtl_mods\c_stdlib.h"

#endif
