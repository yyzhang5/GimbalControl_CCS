/*=====================================================================================
 File name:        PARK.C                   
 Description:  Park Transformation               

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 05-07-2013	Version 1.0
-------------------------------------------------------------------------------------*/

#include "park.h"
#include "math.h"

void park_calc(PARK *v)
{	
	float Cosine,Sine;

	Cosine = cos(v->Angle);
	Sine = sin(v->Angle);
	v->Ds = v->Alpha*Cosine + v->Beta*Sine;
	v->Qs = v->Beta*Cosine - v->Alpha*Sine;

}


