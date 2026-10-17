/* =================================================================================
File name:       PID_REG.H                      
Description: 
Header file containing constants, data type definitions, and 
function prototypes for the PIDREG.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 05-07-2013	Version 1.0
------------------------------------------------------------------------------*/
#ifndef __PIDREG_H__
#define __PIDREG_H__

typedef struct {  float  Ref;   			// Input: Reference input 
				  float  Fdb;   			// Input: Feedback input 
				  float  Err;				// Variable: Error
				  float  Err_1;				// Variable: Error_1
				 
				  float  Kp;				// Parameter: Proportional gain
				  float  Ki;			    // Parameter: Integral gain
				  float  Kd; 		        // Parameter: Derivative gain
				  float  Up;				// Variable: Proportional output 
				  float  Ui;				// Variable: Integral output 
				  float  Ui_1;				
				  
				  float  Ud;				// Variable: Derivative output 	
				  float  OutPreSat; 		// Variable: Pre-saturated output
				  float  OutMax;		    // Parameter: Maximum output 
				  float  OutMin;	    	// Parameter: Minimum output
				  
				  float  UiMax;		        // Parameter: Maximum  Integral output 
				  float  UiMin;	         	// Parameter: Minimum  Integral output
				  float  Out;   			// Output: PID output 

		 	 	  
		 	 	  void  (*calc)();		  	// Pointer to calculation function
				 } PIDREG;	            

typedef PIDREG *PIDREG_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG3 object.
-----------------------------------------------------------------------------*/                     
#define PIDREG_DEFAULTS_iq { 0,0,0,0,\
                             0.06,0.0003,0,0,0,0, \
                             0,0,0.7,-0.7, \
                             0.92,-0.92,0, \
						     (void (*)(unsigned int))pid_reg_calc }//ki=0.0003
#define PIDREG_DEFAULTS_id { 0,0,0,0, \
                             0.06,0,0,0, \
                             0,0,0.3,-0.3, \
                             0.3,-0.3,0,0.0,0, \
						     (void (*)(unsigned int))pid_reg_calc }//ki=0.00001
//M1_speed原限幅均为1.5
#define PIDREG_DEFAULTS_speed_M1 { 0,0,0,0,\
                                0.0001,0.00000005,0.0,0,0,0,\
                                0,0,1,-1,\
                                0.4,-0.4,0,\
						       (void (*)(unsigned int))pid_reg_calc }
#define PIDREG_DEFAULTS_picture_M1 { 0,0,0,0,\
                                0.0001,0.00000005,0.0,0,0,0,\
                                0,0,1,-1,\
                                0.05,-0.05,0,\
                               (void (*)(unsigned int))pid_reg_calc }
//M2_speed原限幅50
#define PIDREG_DEFAULTS_speed_M2 { 0,0,0,0,\
                                10,0.004,0.85,0,0,0,\
                                0,0,1,-1,\
                                0.8,-0.8,0,\
						       (void (*)(unsigned int))pid_reg_calc_M2 }
#define PIDREG_DEFAULTS_pos_M1 { 0,0,0,0,\
                                0.2,0.001,0,0,0,0,\
                                0,0,10,-10,\
                                10,-10,0,\
						       (void (*)(unsigned int))pid_reg_pos_calc }
#define PIDREG_DEFAULTS_pos_M2 { 0,0,0,0,\
                                0.8,0.001,0,0,0,0,\
                                0,0,10,-10,\
                                10,-10,0,\
						       (void (*)(unsigned int))pid_reg_pos_calc_M2 }
#define PIDREG_DEFAULTS_picture_M2 { 0,0,0,0,\
                                0.0001,0.00000005,0.0,0,0,0,\
                                0,0,1,-1,\
                                1,-1,0,\
                               (void (*)(unsigned int))pid_reg_calc }
/*------------------------------------------------------------------------------
Prototypes for the functions in PIDREG3.C
------------------------------------------------------------------------------*/
void pid_reg_calc(PIDREG_handle);
void pid_reg_calc_M2(PIDREG_handle);
void pid_reg_pos_calc(PIDREG_handle);
void pid_reg_pos_calc_M2(PIDREG_handle);

#endif // __PIDREG3_H__
