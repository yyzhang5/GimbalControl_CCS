/*=====================================================================================
 File name:        SVGEN.C

 Description:  Space-vector PWM generation based on d-q components                    

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 05-07-2013	Version 1.0
-------------------------------------------------------------------------------------*/
#include "svgen.h"
void svgen_calc(SVGEN *v)
{	

	float Va,Vb,Vc,t1,t2;
	unsigned int Sector = 0; 
		 
// Inverse clarke transformation
	Va = v->Ubeta;
    Vb = -0.5 * v->Ubeta + 0.8660254 * v->Ualpha;		// 0.8660254 = sqrt(3)/2 
    Vc = -0.5 * v->Ubeta - 0.8660254 * v->Ualpha;		// 0.8660254 = sqrt(3)/2

// 60 degree Sector determination
	if (Va>0)
		Sector = 1; 
	if (Vb>0)
		Sector = Sector + 2;
	if (Vc>0)   
		Sector = Sector + 4;
       
// X,Y,Z (Va,Vb,Vc) calculations
	Va = v->Ubeta;										// X = Va 
	Vb = 0.5 * v->Ubeta + 0.8660254 * v->Ualpha;		// Y = Vb 
	Vc = 0.5 * v->Ubeta - 0.8660254 * v->Ualpha;		// Z = Vc
    
	if (Sector==0)  // Sector 0: this is special case for (Ualpha,Ubeta) = (0,0)
	{
		v->Ta = 0.5;
		v->Tb = 0.5;
		v->Tc = 0.5;
	}
	else if (Sector==1)  // Sector 1: t1=Z and t2=Y (abc ---> Tb,Ta,Tc),U60,U120
	{
		t1 = Vc;
		t2 = Vb;
		if(t1+t2 > 1)
		{
			t1 = t1/(t1+t2);
			t2 = 1-t1;				
		}		
		v->Tb = 0.5*(1-t1-t2);			// tbon = (1-t1-t2)/2
		v->Ta = v->Tb+t1;				// taon = tbon+t1
		v->Tc = v->Ta+t2;				// tcon = taon+t2
    }
	else if (Sector==2)  // Sector 2: t1=Y and t2=-X (abc ---> Ta,Tc,Tb),U300,U0
	{
		t1 = Vb;
		t2 = -Va;
		if(t1+t2 > 1)
		{
			t1 = t1/(t1+t2);
			t2 = 1-t1;				
		}	       
       v->Ta = 0.5*(1-t1-t2);			// taon = (1-t1-t2)/2
       v->Tc = v->Ta+t1;				// tcon = taon+t1
       v->Tb = v->Tc+t2;				// tbon = tcon+t2
	}      
	else if (Sector==3)  // Sector 3: t1=-Z and t2=X (abc ---> Ta,Tb,Tc),U0,U60
	{
		t1 = -Vc;
		t2 = Va;
		if(t1+t2 > 1)
		{
			t1 = t1/(t1+t2);
			t2 = 1-t1;				
		}		
		v->Ta = 0.5*(1-t1-t2);			// taon = (1-t1-t2)/2
		v->Tb = v->Ta+t1;				// tbon = taon+t1
		v->Tc = v->Tb+t2;				// tcon = tbon+t2
	}   
	else if (Sector==4)  // Sector 4: t1=-X and t2=Z (abc ---> Tc,Tb,Ta),U180,U240
	{
		t1 = -Va;
		t2 = Vc;
		if(t1+t2 > 1)
		{
			t1 = t1/(t1+t2);
			t2 = 1-t1;				
		}		
		v->Tc = 0.5*(1-t1-t2);			// tcon = (1-t1-t2)/2
		v->Tb = v->Tc+t1;				// tbon = tcon+t1
		v->Ta = v->Tb+t2;				// taon = tbon+t2
	}   
	else if (Sector==5)  // Sector 5: t1=X and t2=-Y (abc ---> Tb,Tc,Ta),U120,U180
	{
		t1 = Va;
		t2 = -Vb;
		if(t1+t2 > 1)
		{
			t1 = t1/(t1+t2);
			t2 = 1-t1;				
		}       
		v->Tb = 0.5*(1-t1-t2);			// tbon = (1-t1-t2)/2
		v->Tc = v->Tb+t1;				// tcon = tbon+t1
		v->Ta = v->Tc+t2;				// taon = tcon+t2
    }   
	else if (Sector==6)  // Sector 6: t1=-Y and t2=-Z (abc ---> Tc,Ta,Tb),U0,U60
	{
		t1 = -Vb;
		t2 = -Vc;
		if(t1+t2 > 1)
		{
			t1 = t1/(t1+t2);
			t2 = 1-t1;				
		}   		
		v->Tc = 0.5*(1-t1-t2);			// tcon = (1-t1-t2)/2
		v->Ta = v->Tc+t1;				// taon = tcon+t1
		v->Tb = v->Ta+t2;				// tbon = taon+t2 
	}
    
// Convert the unsigned format (ranged (0,1)) -> signed format (ranged (-1,1))
//    v->Ta = 2 * (v->Ta - 0.5);
//    v->Tb = 2 * (v->Tb - 0.5);
//    v->Tc = 2 * (v->Tc - 0.5);
}


