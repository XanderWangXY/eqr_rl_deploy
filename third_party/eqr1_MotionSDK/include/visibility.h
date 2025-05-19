
#ifndef EHR_APP_CORE_VISIBILITY_H_
#define EHR_APP_CORE_VISIBILITY_H_

// __GNUC__ doesn’t mean GCC specifically. All compilers that support GNU C extensions define it, including clang and ICC.
#if defined _WIN32 || defined __CYGWIN__
    #ifdef BUILDING_DLL
        #ifdef __GNUC__
            #define PUBLIC_API __attribute__ ((dllexport))
        #else
            #define PUBLIC_API __declspec(dllexport) // Note: actually gcc seems to also supports this syntax.
        #endif
    #else
        #ifdef __GNUC__
            #define PUBLIC_API __attribute__ ((dllimport))
        #else
            #define PUBLIC_API __declspec(dllimport) // Note: actually gcc seems to also supports this syntax.
        #endif
    #endif
    #define LOCAL_API
#else
    #if __GNUC__ >= 4
        #define PUBLIC_API __attribute__ ((visibility ("default")))
        #define LOCAL_API  __attribute__ ((visibility ("hidden")))
    #else
        #error "##### requires gcc version >= 4.0 #####"
        #define PUBLIC_API
        #define LOCAL_API
    #endif
#endif

#endif // EHR_APP_CORE_VISIBILITY_H_
