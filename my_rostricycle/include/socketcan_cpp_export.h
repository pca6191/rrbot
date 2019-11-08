#ifndef SOCKETCAN_CPP_EXPORT_H
#define SOCKETCAN_CPP_EXPORT_H

#ifdef SOCKETCAN_CPP_STATIC_DEFINE
#  define SOCKETCAN_CPP_EXPORT
#  define SOCKETCAN_CPP_NO_EXPORT
#else
#  ifndef SOCKETCAN_CPP_EXPORT
#    ifdef socketcan_cpp_EXPORTS
        /* We are building this library */
#      define SOCKETCAN_CPP_EXPORT 
#    else
        /* We are using this library */
#      define SOCKETCAN_CPP_EXPORT 
#    endif
#  endif

#  ifndef SOCKETCAN_CPP_NO_EXPORT
#    define SOCKETCAN_CPP_NO_EXPORT 
#  endif
#endif

#ifndef SOCKETCAN_CPP_DEPRECATED
#  define SOCKETCAN_CPP_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef SOCKETCAN_CPP_DEPRECATED_EXPORT
#  define SOCKETCAN_CPP_DEPRECATED_EXPORT SOCKETCAN_CPP_EXPORT SOCKETCAN_CPP_DEPRECATED
#endif

#ifndef SOCKETCAN_CPP_DEPRECATED_NO_EXPORT
#  define SOCKETCAN_CPP_DEPRECATED_NO_EXPORT SOCKETCAN_CPP_NO_EXPORT SOCKETCAN_CPP_DEPRECATED
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define SOCKETCAN_CPP_NO_DEPRECATED
#endif

#define HAVE_SOCKETCAN_HEADERS  // 取消本定義，可以跳過 init linux driver，純 debug 邏輯

#endif
