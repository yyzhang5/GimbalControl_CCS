///* =================================================================================
//File name:       CLARKE.H
//===================================================================================*/
//
//
//#ifndef __CLARKE_H__
//#define __CLARKE_H__
//
//typedef struct {  _iq  As;  		// Input: phase-a stator variable
//				  _iq  Bs;			// Input: phase-b stator variable
//				  _iq  Cs;			// Input: phase-c stator variable
//				  _iq  Alpha;		// Output: stationary d-axis stator variable
//				  _iq  Beta;		// Output: stationary q-axis stator variable
//		 	 	} CLARKE;
//
///*-----------------------------------------------------------------------------
//	Default initalizer for the CLARKE object.
//-----------------------------------------------------------------------------*/
//#define CLARKE_DEFAULTS { 0, \
//                          0, \
//                          0, \
//                          0, \
//                          0, \
//              			}
//
///*------------------------------------------------------------------------------
//	CLARKE Transformation Macro Definition
//------------------------------------------------------------------------------*/
//
////  1/sqrt(3) = 0.57735026918963
//#define  ONEbySQRT3   0.57735026918963    /* 1/sqrt(3) */
//
//
//// Clarke transform macro (with 2 currents)
////==========================================
//#define CLARKE_MACRO(v)										\
//v.Alpha = v.As;												\
//v.Beta = _IQmpy((v.As +_IQmpy2(v.Bs)),_IQ(ONEbySQRT3));
//
//
//// Clarke transform macro (with 3 currents)
////==========================================
//#define CLARKE1_MACRO(v)									\
//v.Alpha = v.As;											    \
//v.Beta  = _IQmpy((v.Bs - v.Cs),_IQ(ONEbySQRT3));
//
//#endif // __CLARKE_H__
//

/* =================================================================================
File name:       CLARKE.H
Description:
Header file containing constants, data type definitions, and
function prototypes for the CLARKE.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 05-07-2013 Version 1.0
------------------------------------------------------------------------------*/

typedef struct {  float  As;    // Input: phase-a stator variable
                  float  Bs;    // Input: phase-b stator variable
                  float  Alpha; // Output: stationary d-axis stator variable
                  float  Beta;  // Output: stationary q-axis stator variable
                  void  (*calc)();  // Pointer to calculation function
                 } CLARKE;

typedef CLARKE *CLARKE_handle;

/*-----------------------------------------------------------------------------
Default initalizer for the CLARKE object.
-----------------------------------------------------------------------------*/                     
#define CLARKE_DEFAULTS { 0, \
                          0, \
                          0, \
                          0, \
                          (void (*)(unsigned int))clarke_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in CLARKE.C
------------------------------------------------------------------------------*/
void clarke_calc(CLARKE_handle);
