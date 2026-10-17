/*=====================================================================================
 File name:        IPARK.C                   
 Description:  Inverse Park Transformation               

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 05-07-2013	Version 1.0
-------------------------------------------------------------------------------------*/

#include "ipark.h"
#include "math.h"

void ipark_calc(IPARK *v)
{	
	float Cosine,Sine;
   
	Cosine = cos(v->Angle);
	Sine = sin(v->Angle);
	v->Alpha = v->Ds*Cosine - v->Qs*Sine;
	v->Beta = v->Qs*Cosine + v->Ds*Sine;  
}
