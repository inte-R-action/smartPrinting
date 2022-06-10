/**
 * @file print_planning.h
 * @author Arya shabani 
 * @brief  this hearder file provides functions for planning of print 
 * @version V 1.2 
 * @date 2022-04-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef LCD_H
     # define LCD_H
     # include "src/ARYA/Display_LCD.h"
#endif
#ifndef STEPPER_H
     # define STEPPER_H
     # include "src/ARYA/stepper_motion.h"
#endif
# include"src/ARYA/control_printer.h"
# include "src/ARYA/Fan.h"
# include "src/ARYA/Set_temprature.h"
  
 // Allocate timer3 for motion control during printing 
 TIM_TypeDef *inst = TIM3; // this a strucuter for timer variables (http://stm32.kosyak.info/doc/annotated.html)
 HardwareTimer *Xaxis_timer = new HardwareTimer(inst); // create instace of hardware class

 // Allocate timer4 for monitoring tempreture interrupt 
 TIM_TypeDef *inst3 = TIM2; // this a strucuter for timer variables (http://stm32.kosyak.info/doc/annotated.html)
 HardwareTimer *extruder_timer = new HardwareTimer(inst3); // create instace of hardware clas

  // Allocate timer4 for monitoring tempreture interrupt 
 TIM_TypeDef *inst4 = TIM7; // this a strucuter for timer variables (http://stm32.kosyak.info/doc/annotated.html)
 HardwareTimer *Yaxis_timer = new HardwareTimer(inst4); // create instace of hardware clas

 stepper_motion Ender3;

 int countery=0, counterx=0, counterE=0;
 float stepE=0;

 void x_motion_callback()
 {
   Ender3.Move_axis_onestep('x');
   counterx++; 
 }


 void y_motion_callback()
 {
   Ender3.Move_axis_onestep('y');
   countery++;
  }
 
 void E_motion_callback(){
   if (counterE<stepE)
   {
      Ender3.move_extruder_onestep();
      counterE++;
   }
   
}

/*
* This function move all axes to their 
*  home position
*
*/
void Go_Home()
{

   while((Ender3.endX == LOW))
   {
     Ender3.move_axis_mm(10, 'x', Backward);
   }

     while((Ender3.endY == LOW))
   {
      Ender3.move_axis_mm(10, 'y', Backward);
   }

    while((Ender3.endZ == LOW))
   {
      Ender3.move_axis_mm(10, 'z', Backward);
   }

}
  






/**
 * @brief: this function moves the 3D printer nozzle with extrusion 
 *        or without extrusion. is extrusion value is zero the nozzle
 *        moves without extrusion. Generally this function uses three timer interrupts.
 *        the bigger vbalue among x and y as the master interrupt 
 *        and two others as followers that are trigerd slower than the master
 * note::speed of porint and speed on extrusion should be set before this function 
 * 
 * @param x :  x direction movement in mm
 * @param y :  y direction movement in mm
 * @param dist_extrude : the mount of extrusion 
 */
void Move_XYaxis_simply_with_extrusion(float x, float y, float dist_extrude)
{
  // Initialization 
   bool extrusion=false;
   float stepx=0, stepy=0;
   float T_tot=0, start_time=0, Freqy=0, FreqE=0, Freqx=0, T_y=0, T_x=0;
   char axis;
   counterx=0;
   countery=0;
   counterE=0;
   stepE=0;

   // convert extrusion distance(mm)->degree and compute number of steps
   if (dist_extrude!=0)
   {
      extrusion=true; 
      Ender3.Direction_selection(Forward,'E'); 
      stepE = Ender3.mm_to_deg(dist_extrude, 'E')/Ender3.step_angle; 
      FreqE=(1/Ender3.PulseE)*1000000;
   }


   // A) motion is  in X-direction
   if ( (x!=0) & (y==0))
   { 
      axis='x'; 

     // determine the direction of motion based on X sign 
     if (x<0)
     {
       Ender3.Direction_selection(Backward, axis);
       x=abs(x);
     }
     else if (x>0)
     {
      Ender3.Direction_selection(Forward, axis);     
      x=abs(x);
     }
     
     // convert X(mm)->degree and compute numebr of steps
     stepx = Ender3.mm_to_deg(x, axis)/Ender3.step_angle; 
    
      /*   
        set interrupts for motion and extrusion
      */
      if (extrusion==true)
      {
        T_tot= stepx*Ender3.PulseX; // micro second
        Freqx=(1/Ender3.PulseX)*1000000;

        // initialized the intrrupt for temp monitoring of hot end
        Xaxis_timer->setOverflow(Freqx, HERTZ_FORMAT);
        Xaxis_timer->attachInterrupt(x_motion_callback);  

        extruder_timer->setOverflow(FreqE, HERTZ_FORMAT);
        extruder_timer->attachInterrupt(E_motion_callback);
      }
    }


   // B) motion is only in Y direction 
   else if((y!=0) & (x==0))
   {
     axis='y'; 
     // determine the direction of motion based on Y sign 
     if (y<0)
     {
       Ender3.Direction_selection(Backward, axis);
       y=abs(y);
     }
     else if (y>0)
     {
       Ender3.Direction_selection(Forward, axis);
       y=abs(y);
     }
     
     // convert movment-Y(mm)->degree and compute numebr of steps
     stepy = Ender3.mm_to_deg(y, axis)/Ender3.step_angle; 
    
      /*   
        set interrupts for motion and extrusion
      */
      if (extrusion==true)
      {
        T_tot= stepy*Ender3.PulseY; // micro second
        Freqy=(1/Ender3.PulseY)*1000000;

        Yaxis_timer->setOverflow(Freqy, HERTZ_FORMAT);
        Yaxis_timer->attachInterrupt(y_motion_callback);

        extruder_timer->setOverflow(FreqE, HERTZ_FORMAT);
        extruder_timer->attachInterrupt(E_motion_callback);
      } 
    }


   // C) motion is in both directions 
   else if ((x!=0) & (y!=0))
   {
     // double axis
      axis='D';
        
     // determine the direction of motion based on X sign 
     if (x<0)
     {
       Ender3.Direction_selection(Backward, 'x');
       x=abs(x);
     }
     else if (x>0)
     {
      Ender3.Direction_selection(Forward, 'x');     
      x=abs(x);
     }

      // determine the direction of motion based on Y sign 
     if (y<0)
     {
      Ender3. Direction_selection(Backward, 'y');
       y=abs(y);
     }
     else if (y>0)
     {
       Ender3.Direction_selection(Forward, 'y');
       y=abs(y);
     }
     

      // convert movment-X(mm)->degree and compute numebr of steps
      stepx = Ender3.mm_to_deg(x, 'x')/Ender3.step_angle; 

      // convert movment-Y(mm)->degree and compute numebr of steps
      stepy = Ender3.mm_to_deg(y, 'y')/Ender3.step_angle; 



      /*   
        set interrupts for motion and extrusion
        if x is the leading axis 
      */
     if (x >= y)
     {
        T_tot= stepx*Ender3.PulseX; // micro second
        T_y= T_tot/stepy;
        Freqx=(1/Ender3.PulseX)*1000000;
        Freqy=(1/T_y)*1000000;

        // initialized the intrrupt for temp monitoring of hot end
        Xaxis_timer->setOverflow(Freqx, HERTZ_FORMAT);
        Xaxis_timer->attachInterrupt(x_motion_callback);

        Yaxis_timer->setOverflow(Freqy, HERTZ_FORMAT);
        Yaxis_timer->attachInterrupt(y_motion_callback);

        extruder_timer->setOverflow(FreqE, HERTZ_FORMAT);
        extruder_timer->attachInterrupt(E_motion_callback);
  
      }

      /*   
        set interrupts for motion and extrusion
        if y is the leading axis 
      */

     else if (y > x)
     {
        T_tot= stepy*Ender3.PulseY; // micro second
        T_x = T_tot/stepx;
        Freqy=(1/Ender3.PulseY)*1000000;  
        Freqx=(1/T_x)*1000000;

        // initialized the intrrupt for temp monitoring of hot end
        Xaxis_timer->setOverflow(Freqx, HERTZ_FORMAT);
        Xaxis_timer->attachInterrupt(x_motion_callback);

        Yaxis_timer->setOverflow(Freqy, HERTZ_FORMAT);
        Yaxis_timer->attachInterrupt(y_motion_callback);

        extruder_timer->setOverflow(FreqE, HERTZ_FORMAT);
        extruder_timer->attachInterrupt(E_motion_callback);
     
      }
    }


   // activate interrupts 
   switch (axis)
  {

   case 'x':
      if (extrusion==true)
      {
        Xaxis_timer->refresh();
        Xaxis_timer->resume();
        extruder_timer->refresh();
        extruder_timer->resume();
        start_time=micros();
        while ((micros()- start_time) < T_tot);
        Xaxis_timer->detachInterrupt();
        extruder_timer->detachInterrupt();

      }

      else if(extrusion==false)
      {
         for (int i = 0; i < stepx; i++)
        {
          Ender3.Move_axis_onestep(axis);
          delayMicroseconds(Ender3.PulseX);  
        }
      }
    break;


   case 'y':

      if (extrusion==true)
      {
        Yaxis_timer->refresh();
        Yaxis_timer->resume();
        extruder_timer->refresh();
        extruder_timer->resume();
        start_time=micros();
        while ((micros()- start_time) < T_tot);
        Yaxis_timer->detachInterrupt();
        extruder_timer->detachInterrupt();

      }

      else if(extrusion==false)
      {
         for (int i = 0; i < stepy; i++)
        {
         Ender3.Move_axis_onestep(axis); 
         delayMicroseconds(Ender3.PulseY);
        }
      }
    break; 


   case 'D':
      if (extrusion==true)
      {
        Yaxis_timer->refresh();
        Xaxis_timer->refresh();
        Xaxis_timer->resume();
        Yaxis_timer->resume();
        extruder_timer->refresh();
        extruder_timer->resume();
        start_time=micros();
        while ((micros()- start_time) < T_tot);
        Yaxis_timer->detachInterrupt();
        Xaxis_timer->detachInterrupt();
        extruder_timer->detachInterrupt();

      }

      else if(extrusion==false)
      {
        Yaxis_timer->refresh();
        Xaxis_timer->refresh();
        Xaxis_timer->resume();
        Yaxis_timer->resume();
        start_time=micros();
        while ((micros()- start_time) < T_tot);
        Yaxis_timer->detachInterrupt();
        Xaxis_timer->detachInterrupt();
      }
    break; 
   
   
    default:
    break;
  }  
   
}



 void print_init()
 {
   Ender3.setFeedrate(10);
   Ender3.set_Speed(33.3,'x',true);
   Ender3.set_Speed(33.3,'y',true);
   Ender3.set_Speed(33.3,'z',true);

   display_string("heating_end", 10, 32);
   preheating_hotend();
   display_string("heating_bed", 10, 32);
   preheating_bed();
   display_string("ready", 10, 32);

   Go_Home();

   Move_XYaxis_simply_with_extrusion(0, 90, 10);
   Move_XYaxis_simply_with_extrusion(0, -90, 10);
  }



/**
 *  This function set the tarpezoid velocity profile for the given distance 
 *  in general Vm, am are equal to maximum velocity and acceleration 
 *  of the motors that are set in the class. In the case that D is less than a criteria 
 *  Vm and am are needed to reduce otherwise there isn't any trapezoid that meet the 
 *  desired properties. 
 *     Vm  __________________________
 *        /|                        |\     
 *       / |                        | \   
 *   am /  |                        |  \ am 
 *     /   |                        |   \ 
 *    /    |                        |    \
  Vo +-----+------------------------+-----+-----
     | ta  |         ts             |  ta |
     <-----------------D------------------>
     | Da  |           Ds           | Da  |
     inputs:
          D--> distance to travel 
          Vo --> initial velocity 
     outputs:
          Vm --> maximum velocity of the trapezoid 
          am--> accelation for constant accelarion section of the motion 
          t_rise --> ta
          t_steady--> ts
          D_raise--> Da
          D_steady--> Ds


void printer_motion::set_trapezoid_profile(float D, float V0, float(&Vm),  float(&D_raise), float(&D_steady))
    
{
   // Raise time based on maximum velocity and acceleration of the class
    float t_raise = (V_max-V0)/a_max; 
    float t_steady=0;
     
     

   // If the distance is less than @ (Vm-Vo)*t_rise @, there isn't any trapezoid profile 
   // that can be fitted with the required maximum velocity and acceleration for the given distance
   if ( D < (V_max-V0)*t_raise )
   {
     // if we only update Vm and leave am intact, in many cases t_rise
     // decreases to the mount that the velocity profil approaches to the pluse signal 
     // shape thus with any D below than the limit 'am' also will be updated in order
     // to maintain the t_rise to the initial value  
     Vm = D + (V0*t_raise);
    //am = (Vm-V0)/t_raise;
     t_steady = ( D - ((Vm-V0)*t_raise) )/Vm;
     D_raise = 0.5*(Vm-V0)*t_raise;
     D_steady = Vm*t_steady;
   }

   else
   {
     
      Vm =V_max;
    //am = a_max; 
      t_steady = ( D - ((Vm-V0)*t_raise) )/Vm;
      D_raise = 0.5*(Vm-V0)*t_raise;
      D_steady = Vm*t_steady;
 
   }



   
}



*   This function move a given axis in a given direction with 
*   trapezoid velocity profile in which Vmax is set through 
*   the class features and V0 will be recived be user
*   inputs: 
*           dist--> distance to go   
*           dir --> direction to go 
*           V0--> initial velocity of trapezoid 
*           axis--> axis to move 


void printer_motion::Move_with_trapeziod_profile(float dist, int dir, float V0,  char axis)
{
  // distancers in mm, velocity mm/s, acceleratio in mm/s2
  float teta=0,teta_raise=0, teta_steady=0, D_steady=0, D_raise=0; 
  float Vm, W=0,Wm=0,W0=0;
  int step_tot=0, step_raise=0, step_steady=0;


 // set the trapezoid velocity profile with the maximum velocity of the class 
 // and given V0
    set_trapezoid_profile(dist, V0, Vm, D_raise, D_steady);


 // convert mm to deg
    teta=mm_to_deg(dist, axis); 
    teta_raise=mm_to_deg(D_raise, axis);
    teta_steady=mm_to_deg(D_steady, axis); 


 // compute required steps for each part of the tarpezoid
     step_tot= teta/step_angle;      // steps require for whole path
     step_raise= teta_raise/step_angle; // steps for constant acceleration motion section 
     step_steady=teta_steady/step_angle;// steps for constant velocity section

 // converting teta to step sometimes create rounding error
     if ((2*step_raise + step_steady) != step_tot)
     {
       int step_er = step_tot - (2*step_raise + step_steady);
       step_steady = step_steady + step_er;
     }
         
  // converting linear to angular velocity  
    W0= linear_to_angular_velocity(V0,axis);
    Wm= linear_to_angular_velocity(Vm,axis);
  
 // ascending event from W0 --> Wm in number of steps equal to step_raise 
    for (int i = 0; i < step_raise; i++)
   {
     W = W0 + i*((Wm-W0)/step_raise); 
     set_Speed_RPM(W,axis);
     Move_axis_onestep(dir,axis);
   }

 // constant velocity event Wm
    for (int i = 0; i < step_steady; i++)
   {
     set_Speed_RPM(Wm,axis);
     Move_axis_onestep(dir,axis);
   }
 
 // descending event from Wm --> W0 in number of steps equal to step_raise 
      for (int i = 0; i < step_raise; i++)
   {
      W = Wm + (i+1)*((W0-Wm)/step_raise); 
      set_Speed_RPM(W,axis);
      Move_axis_onestep(dir,axis);
   }

}





*/