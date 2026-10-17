/*=====================================================================================
 File name:        PID_REG.C
                    
 Originator:	Digital Control Systems Group
			Texas Instruments

 Description:  The PID controller with anti-windup

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 08-28-2012	Version 1.0
-------------------------------------------------------------------------------------*/

#include "pid_reg.h"
#include "math.h"

void pid_reg_calc(PIDREG *v)
{	
	 // Compute the error
	    v->Err = v->Ref - v->Fdb;
	    // Compute the proportional output
	    v->Up = v->Kp * v->Err;
	    // Compute the integral output
	    v->Ui = v->Ui_1 +  v->Ki * v->Err;
		if(v->Ui >  v->UiMax)
		{
			v->Ui = v->UiMax;
		}
		else if(v->Ui < v->UiMin)
		{
			v->Ui = v->UiMin;
		}
	    // Compute the derivative output
	    v->Ud = v->Kd * (v->Err - v->Err_1);
	    // Compute the pre-saturated output
	    v->OutPreSat = v->Up + v->Ui + v->Ud;
	    // Saturate the output
		if (v->OutPreSat > v->OutMax)
		{
			v->Out =  v->OutMax;
		}
		else if (v->OutPreSat < v->OutMin)
		{
			v->Out =  v->OutMin;
		}
		else
		{
			v->Out = v->OutPreSat;
		}
	    // Update the previous proportional output
		v->Err_1 = v->Err;
		v->Ui_1 = v->Ui;
}
void pid_reg_calc_M2(PIDREG *v)
{
     // Compute the error
        v->Err = v->Ref - v->Fdb;
        // Compute the proportional output
        v->Up = v->Kp * v->Err;
        // Compute the integral output
        v->Ui = v->Ui_1 +  v->Ki * v->Err;
        if(v->Ui >  v->UiMax)
        {
            v->Ui = v->UiMax;
        }
        else if(v->Ui < v->UiMin)
        {
            v->Ui = v->UiMin;
        }
        // Compute the derivative output
        v->Ud = v->Kd * (v->Err - v->Err_1);
        // Compute the pre-saturated output
        v->OutPreSat = v->Up + v->Ui + v->Ud;
        // Saturate the output
        if (v->OutPreSat > v->OutMax)
        {
            v->Out =  v->OutMax;
        }
        else if (v->OutPreSat < v->OutMin)
        {
            v->Out =  v->OutMin;
        }
        else
        {
            v->Out = v->OutPreSat;
        }
        // Update the previous proportional output
        v->Err_1 = v->Err;
        v->Ui_1 = v->Ui;
}
void pid_reg_pos_calc(PIDREG *v)
{
     // Compute the error
        v->Err = v->Ref - v->Fdb;
        if(v->Err > 180)
            v->Err = v->Err-360;
        if(v->Err < -180)
              v->Err = v->Err+360;

        // Compute the proportional output
        v->Up = v->Kp * v->Err;

        // Compute the integral output
        v->Ui = v->Ui_1 +  v->Ki * v->Err;
        if(v->Ui >  v->UiMax)
        {
            v->Ui = v->UiMax;
        }
        else if(v->Ui < v->UiMin)
        {
            v->Ui = v->UiMin;
        }
        // Compute the derivative output
        v->Ud = v->Kd * (v->Err - v->Err_1);
        // Compute the pre-saturated output
        v->OutPreSat = v->Up + v->Ui + v->Ud;
        // Saturate the output
        if (v->OutPreSat > v->OutMax)
        {
            v->Out =  v->OutMax;
        }
        else if (v->OutPreSat < v->OutMin)
        {
            v->Out =  v->OutMin;
        }
        else
        {
            v->Out = v->OutPreSat;
        }
        // Update the previous proportional output
        v->Err_1 = v->Err;
        v->Ui_1 = v->Ui;
}
void pid_reg_pos_calc_M2(PIDREG *v)
{
     // Compute the error
        v->Err = v->Ref - v->Fdb;
        if(v->Err > 180)
            v->Err = v->Err-360;
        if(v->Err < -180)
              v->Err = v->Err+360;

        // Compute the proportional output
        v->Up = v->Kp * v->Err;

        // Compute the integral output
        v->Ui = v->Ui_1 +  v->Ki * v->Err;
        if(v->Ui >  v->UiMax)
        {
            v->Ui = v->UiMax;
        }
        else if(v->Ui < v->UiMin)
        {
            v->Ui = v->UiMin;
        }
        // Compute the derivative output
        v->Ud = v->Kd * (v->Err - v->Err_1);
        // Compute the pre-saturated output
        v->OutPreSat = v->Up + v->Ui + v->Ud;
        // Saturate the output
        if (v->OutPreSat > v->OutMax)
        {
            v->Out =  v->OutMax;
        }
        else if (v->OutPreSat < v->OutMin)
        {
            v->Out =  v->OutMin;
        }
        else
        {
            v->Out = v->OutPreSat;
        }
        // Update the previous proportional output
        v->Err_1 = v->Err;
        v->Ui_1 = v->Ui;
}
