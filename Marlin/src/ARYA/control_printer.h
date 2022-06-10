#ifndef MATH_H
     # define MATH_H
     # include <math.h>
#endif


#ifndef STEPPER_H
     # define STEPPER_H
     # include "src/ARYA/stepper_motion.h"
#endif


struct extrusion
{
    float velocity=0;
    float length=0;
};
float R_filament=1.75; 

struct extrusion compute_extrusion_properties(float L, float W, float H, float V_print)
{
   extrusion E;
   E.length= ( ((W-H)*L*H) + (PI*pow(H,2)*L)/2 ) / (PI*pow(R_filament,2));
   float time_total= L* V_print;
   E.velocity = E.length/time_total; 
   return E;
}
