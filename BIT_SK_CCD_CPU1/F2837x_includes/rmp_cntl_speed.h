/* =================================================================================
File name:        RMP_CNTL.H  
===================================================================================*/


#ifndef __RMP_CNTL_SPEED_H__
#define __RMP_CNTL_SPEED_H__

typedef struct { _iq    TargetValue; 	// Input: Target input (pu)
				 Uint32 RampDelayMax;	// Parameter: Maximum delay rate (Q0) - independently with global Q			
		 	 	 _iq    RampLowLimit;	// Parameter: Minimum limit (pu)				  
				 _iq    RampHighLimit;	// Parameter: Maximum limit (pu)
				 Uint32 RampDelayCount; // Variable: Incremental delay (Q0) - independently with global Q	
				 _iq    SetpointValue;	// Output: Target output (pu)				 
				 Uint32 EqualFlag;		// Output: Flag output (Q0) - independently with global Q
				 _iq	Tmp;			// Variable: Temp variable
		  	   } RMPCNTL_SPEED;


/*-----------------------------------------------------------------------------
Default initalizer for the RMPCNTL object.
-----------------------------------------------------------------------------*/                     
#define RMPCNTL_SPEED_DEFAULTS {  0, 		 \
                            1,		 \
                           _IQ(-6000),  \
                           _IQ(6000),   \
                            0,       \
                          	0,       \
                          	0,       \
                          	0,       \
                   		  }

/*------------------------------------------------------------------------------
 	RAMP Controller Macro Definition
------------------------------------------------------------------------------*/

#define RC_SPEED_MACRO(v)																	\
	v.Tmp = v.TargetValue - v.SetpointValue;										\
/*  0.0000305 is resolution of Q15 */												\
if (_IQabs(v.Tmp) >= _IQ(0.001))				    							\
{																					\
	v.RampDelayCount++	;															\
		if (v.RampDelayCount >= v.RampDelayMax)										\
		{																			\
			if (v.TargetValue >= v.SetpointValue)									\
				v.SetpointValue += _IQ(0.001);									\
			else																	\
				v.SetpointValue -= _IQ(0.001);									\
																					\
			v.SetpointValue=_IQsat(v.SetpointValue,v.RampHighLimit,v.RampLowLimit);	\
			v.RampDelayCount = 0;													\
																					\
		}																			\
}																					\
else v.EqualFlag = 0x7FFFFFFF;

#endif // __RMP_CNTL_H__
