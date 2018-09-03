// **********************************************************************
// 
//COMPANY:     Continental Automotive , 
// 
//PROJECT:     ARS400
//    
//CPU:         MPC5674x
// 
//Component:   ARS400_En_Cmp
// 
//MODULNAME:   FullModelElementName
// 
//Description:        Global type definitions for all C types
//Integration notes:  The types defined here are always to be used instead of standard ANSI C types
//Package prefix:     -
// 
// **********************************************************************         


#ifndef glob_type_H
#define glob_type_H

/* #include "Project_specific_TypeSettings.h" JüGi: Commented this include as a quick solution for ALGO to be compilable.
                                                    In general Project_specific_TypeSettings.h must be moved from Rhapsody to the 
                                                    Common folder in order to be available for ALGO build.
*/
/*## package ARS400_SW_Algo_Pkg::CoreComponents_Pkg::Types_Pkg */

/*## class TopLevel::glob_type */
/*#[ ignore */
//**************************************************************************
//glob_type 
//Description:        Global type definitions for all C types
//Integration notes:  The types defined here are always to be used instead of standard ANSI C types
//Package prefix:     -
/*#]*/

#ifdef __cplusplus
extern "C"
{
#endif


#undef _MSC_VER // Hack!!

#if ( (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
//Description: Predefined boolean
//Range:       (0...1)
//Resolution:  1
//Unit:        none
typedef unsigned char boolean;
#endif

#if ( (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
//Description: signed byte
//Range:       (-128...127)
//Resolution:  1
//Unit:        none
typedef signed char sint8;
#endif

#if ( (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
//Description: unsigned byte
//Range:       (0...255)
//Resolution:  1
//Unit:        none
typedef unsigned char uint8;
#endif

#if ( (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
//Description: signed short integer
//Range:       (-32768...32767)
//Resolution:  1
//Unit:        none
typedef signed short sint16;
#endif

#if ( (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
//Description: unsigned short integer
//Range:       (0...65535)
//Resolution:  1
//Unit:        none
typedef unsigned short uint16;
#endif

#if ( (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
//Description: signed long integer
//Range:       (-2147483648...2147483647)
//Resolution:  1
//Unit:        none
typedef signed long sint32;
#endif

#if ( (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
//Description: unsigned long integer
//Range:       (0...4294967295)
//Resolution:  1
//Unit:        none
typedef unsigned long uint32;
#endif

#if ( (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
//Description: signed long long
//Range:       (-2^63...2^63-1)
//Resolution:  1
//Unit:        none
typedef signed long long sint64;
#endif

#if ( (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
//Description: unsigned long long
//Range:       (0...2^64-1)
//Resolution:  1
//Unit:        none
typedef unsigned long long uint64;
#endif

#if ( (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
//Description: 32-bit IEEE 754 floating point number
//Range:       -
//Resolution:  -
//Unit:        none
typedef float float32;
#endif

#if ( (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
//Description: 64-bit IEEE 754 floating point number
//Range:       -
//Resolution:  -
//Unit:        none
typedef double float64;
#endif

//Description: Pointer to a signed 8-bit integer
//Range:       -
//Resolution:  -
//Unit:        none
typedef sint8* psint8;

//Description: Pointer to an unsigned 8-bit integer
//Range:       -
//Resolution:  -
//Unit:        none
typedef uint8* puint8;

//Description: Pointer to a signed 16-bit integer
//Range:       -
//Resolution:  -
//Unit:        none
typedef sint16* psint16;

//Description: Pointer to an unsigned 16-bit integer
//Range:       -
//Resolution:  -
//Unit:        none
typedef uint16* puint16;

//Description: Pointer to a signed 32-bit integer
//Range:       -
//Resolution:  -
//Unit:        none
typedef sint32* psint32;

//Description: Pointer to an unsigned 32-bit integer
//Range:       -
//Resolution:  -
//Unit:        none
typedef uint32* puint32;

//Description: A void pointer
//Range:       -
//Resolution:  -
//Unit:        none
typedef void* pvoid;

#if ( (!defined(DISABLE_LEAST_TYPE_GENERATION)) && (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
//Description: signed byte. Optimized Autosar integer data type, may be used if the correct usage is proven by a formal code review or a static analysis by a validated static analysis tool.
//Range:       (-128...127)
//Resolution:  1
//Unit:        none
typedef signed int sint8_least;
#endif

#if (!defined(__IPL_CANTPP__)) /* can not be tested with Cantata */
  #if ( (!defined(DISABLE_LEAST_TYPE_GENERATION)) && (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
    //Description: unsigned byte. Optimized Autosar integer data type, may be used if the correct usage is proven by a formal code review or a static analysis by a validated static analysis tool.
    //Range:       (0...255)
    //Resolution:  1
    //Unit:        none
    typedef unsigned int uint8_least;
  #endif
#else
  //Description: unsigned byte. Optimized Autosar integer data type, may be used if the correct usage is proven by a formal code review or a static analysis by a validated static analysis tool.
  //Range:       (0...255)
  //Resolution:  1
  //Unit:        none
  typedef unsigned int uint8_least;
#endif

#if ( (!defined(DISABLE_LEAST_TYPE_GENERATION)) && (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
//Description: signed short integer. Optimized Autosar integer data type, may be used if the correct usage is proven by a formal code review or a static analysis by a validated static analysis tool.
//Range:       (-32768...32767)
//Resolution:  1
//Unit:        none
typedef signed int sint16_least;
#endif

#if ( (!defined(DISABLE_LEAST_TYPE_GENERATION)) && (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
//Description: unsigned short integer. Optimized Autosar integer data type, may be used if the correct usage is proven by a formal code review or a static analysis by a validated static analysis tool.
//Range:       (0...65535)
//Resolution:  1
//Unit:        none
typedef unsigned int uint16_least;
#endif

#if ( (!defined(DISABLE_LEAST_TYPE_GENERATION)) && (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
//Description: signed long integer. Optimized Autosar integer data type, may be used if the correct usage is proven by a formal code review or a static analysis by a validated static analysis tool.
//Range:       (-2147483648...2147483647)
//Resolution:  1
//Unit:        none
typedef signed long sint32_least;
#endif

#if ( (!defined(DISABLE_LEAST_TYPE_GENERATION)) && (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) && (!defined(__IPL_CANTPP__)) ) ) )
//Description: unsigned long integer. Optimized Autosar integer data type, may be used if the correct usage is proven by a formal code review or a static analysis by a validated static analysis tool.
//Range:       (0...4294967295)
//Resolution:  1
//Unit:        none
typedef unsigned long uint32_least;
#endif

//Description: Boolean constant false (FALSE = boolean 0)
//Range:       constant (boolean)0
//Resolution:  1
//Unit:        none
/*## attribute b_FALSE */
#ifndef b_FALSE
#define b_FALSE ((boolean)0u)
#endif		/* #ifndef b_FALSE */

//Description: Legacy - use b_FALSE wherever possible.
//Range:       constant (boolean)0
//Resolution:  1
//Unit:        none
/*## attribute FALSE */
#ifndef FALSE
#define FALSE 0u
#endif  	/* #ifndef FALSE */


//Description: Boolean constant true (TRUE = boolean 1)
//Range:       constant (boolean)1
//Resolution:  1
//Unit:        none
/*## attribute b_TRUE */
#ifndef b_TRUE
#define b_TRUE ((boolean)1u)
#endif

//Description: Legacy - use b_TRUE wherever possible
//Range:       constant (boolean)1
//Resolution:  1
//Unit:        none
/*## attribute TRUE */
#ifndef TRUE
#define TRUE 1u
#endif

//Description: Null pointer for initialization / comparison
//Range:       constant void* 0
//Resolution:  1
//Unit:        none
/*## attribute NULL */
#ifndef NULL  
#define NULL ((void *)0)
#endif

//#include "hw_macros.h"
/***    User explicit entries    ***/


#ifdef __cplusplus
}
#endif

#endif
/*********************************************************************
	File Path	: ..\..\..\04_Engineering\_gensrc\rhapsody\glob_type.h
*********************************************************************/

