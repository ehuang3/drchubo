#pragma once

#undef ERROR_STREAM
#undef ERROR_PRINT
#undef DEBUG_STREAM
#undef DEBUG_PRINT

#ifdef MODULE_NAME
#define DEBUG_ID "[" MODULE_NAME " DEBUG] "
#define ERROR_ID "[" MODULE_NAME " ERROR] "
#endif

#ifndef ERROR_ID
#define ERROR_ID "[ERROR] "
#endif

#ifndef DEBUG_ID
#define DEBUG_ID "[DEBUG] "
#endif

#define ERROR_STREAM std::cerr << ERROR_ID
#define ERROR_PRINT(fmt,...) fprintf(stderr, ERROR_ID fmt, ##__VA_ARGS__)
// Debug Macros
#ifdef DEBUG
#define DEBUG_STREAM if(1) std::cerr << DEBUG_ID
#define DEBUG_PRINT(fmt,...) fprintf(stderr, DEBUG_ID fmt, ##__VA_ARGS__)
#else
#define DEBUG_STREAM if(0) std::cerr
#define DEBUG_PRINT(fmt,...)
#endif
