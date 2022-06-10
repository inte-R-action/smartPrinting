
#include "stepper_motion.h"




// constructor of the class
stepper_motion::stepper_motion(){init();}



/**
 * This is initialization function that sets all 
 * pins require for moving stepp motor as an
 * output 
 */
void stepper_motion::init()
{
  // setup pins
  pinMode(X_STEP_PIN,OUTPUT);
  pinMode(Y_STEP_PIN,OUTPUT); 
  pinMode(Z_STEP_PIN,OUTPUT);
  pinMode(X_DIR_PIN,OUTPUT);
  pinMode(Y_DIR_PIN,OUTPUT);
  pinMode(Z_DIR_PIN,OUTPUT); 
  pinMode(E0_DIR_PIN,OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT); 
  
// setup interrupt for mirco switches
  pinMode(X_STOP_PIN,INPUT); 
  pinMode(Y_STOP_PIN,INPUT);
  pinMode(Z_STOP_PIN,INPUT); 
}


/**
 This function converts given distance in mm to degree for each axis(for Ender3) 
    Inputs:
     D -->  distace to travel (mm) 
     axis--> selected axis for conversion    
    Outputs: 
     teta--> converted angle 
**/
float  stepper_motion::mm_to_deg(float D, char axis)
  {  
     float teta;
     switch (axis)
     {
     case 'x':
         // one turn of pulley  (360 deg) corresponds to linear motion equal to teeth*pitch (belt)
          teta = (360*D) / (teeth*P_pulley); 
          return teta;
       break;
       
      case 'y':
         // one turn of pulley  (360 deg) corresponds to linear motion equal to teeth*pitch (belt)
          teta = (360*D) / (teeth*P_pulley);
          return teta; 
        break;

      case 'z':
        // z-axis conversion (lead screw)
           teta= (360*D)/ P_screw;
           return teta;
       break; 
      
      case 'E':
        // E-axis conversion (wheel traved distance)
           teta= (360*D) / (2*PI*R_extruder);
           return teta;
       break;
     
     default:
          teta=0;
          return teta;
       break;
     }
  }





/**
    This function: V [mm/s] --> converts --> W or y(rmp)(ender 3)  
       Inputs:            
          axis--> determines which axis conversion is needed
          V -->  limit of linear velocity of motion for x and y axis is: [0.05,251.9] (mm/s)  
                  and for z axis the limitation is [0.003,14.66] 
       Output:
          W--> angular velocity in RMP 
**/
float stepper_motion::linear_to_angular_velocity(float V, char axis)
{
  float W;
  switch (axis)
  {
  case 'x':
    // linear- angular volocity formulation (w*r=v)
     W = V/R_pulley; // w: (rad/sec) 
     W = W* (30/PI); // rad/sec --> rmp
     return W;  
    break;

  case 'y':
    // linear- angular volocity formulation (w*r=v)
     W = V/R_pulley; // w: (rad/sec) 
     W = W* (30/PI); // rad/sec --> rmp
     return W;
     break;

  case 'z':
     // z-axis speed conversion (lead screw)
     W= (V*2*PI)/ P_screw ;  
     W = W* (30/PI); // rad/sec --> rmp
     return W;
     break;

  default:
     W=0;
     return W;
     break;
  }
  
}



/**
 * @brief set the directio pin of  X,Y,Z and E motor 
 *        to forwrad and backward
 * 
 * @param DIR 
 * @param axis 
 */
void stepper_motion::Direction_selection(int DIR, char axis)
{
  switch (axis)
  {
    case 'x':
       if (DIR==Forward) //HIGH for clockwise
       {
           digitalWrite(X_DIR_PIN, HIGH);
           DIRx=Forward;
       }
       else if (DIR==Backward)//LOW for anticlockwise
       {
           digitalWrite(X_DIR_PIN, LOW);
           DIRx=Backward;
       }    
    break;

    case 'y':

       if (DIR==Forward) //HIGH for clockwise
       {
           digitalWrite(Y_DIR_PIN, HIGH); 
           DIRy=Forward;
       }
       else if (DIR==Backward)//LOW for anticlockwise
       {
           digitalWrite(Y_DIR_PIN, LOW);
           DIRy=Backward;
       }     
    break;

    case 'z':
       if (DIR==Forward) //HIGH for clockwise
       {
           digitalWrite(Z_DIR_PIN, LOW); 
           DIRz=Forward;
       }
       else if (DIR==Backward)//LOW for anticlockwise(toward the collector plate)
       {
           digitalWrite(Z_DIR_PIN, HIGH);
           DIRz=Backward;
       }     
    break;
      
    case 'E':
       if (DIR==Forward) 
       {
           digitalWrite(E0_DIR_PIN, HIGH); //HIGH for clockwise(inward)
       }
       else if (DIR==Backward)
       {
           digitalWrite(E0_DIR_PIN, LOW); //LOW for anticlockwise(outward)
       } 
    break;  
  
    default:
    break;
   }


}

/**
   This function:  moves the stpper motor one step ahead with the pulsed width that should be set  
                   prior to thi function in set_axis_SpeedetSpeed funtion
          Inputs:          
          axis--> select an axis through which the motion is required 
          DIR-->  direction of the motion (forwrad:1 and backward:-1)     
**/ 
void stepper_motion::Move_axis_onestep(char axis)
{
    switch (axis)
    {

    case 'x':

        endX=LOW;
       // check if the x axis microswitch is heated
        MonitorMicroSwitches();

        // the axis may move freely as it doesn't reach to the end 
        // once it reaches the end-axis it can move only forward  
        if ( (endX == LOW) || (endX==HIGH && DIRx==Forward))
        {
          digitalWrite(X_STEP_PIN,LOW);
          digitalWrite(X_STEP_PIN,HIGH); 
        }
      
      break;
    
    case 'y':
         endY =LOW;
         MonitorMicroSwitches();
        if ((endY == LOW) || (endY==HIGH && DIRy==Forward))
        {
           digitalWrite(Y_STEP_PIN,LOW); 
           digitalWrite(Y_STEP_PIN,HIGH);
        }

    break;

    case 'z':
      endZ =LOW;
       
        MonitorMicroSwitches();
        if ((endZ == LOW) || (endZ==HIGH && DIRz==Forward))
        {
           digitalWrite(Z_STEP_PIN,LOW); 
           digitalWrite(Z_STEP_PIN,HIGH);
        }
     
    break;


    default:
      break;
    }





}


/**
    This function monitors all micro switches to see
    when a moving part heat them 
    Note: if microswitch of an axis is not connetct the stepper of the axis will be disabled
**/
void stepper_motion::MonitorMicroSwitches()
{

  if (digitalRead(X_STOP_PIN)== HIGH)
  {
    endX=HIGH; 
  } 

    if (digitalRead(Y_STOP_PIN)== HIGH)
  {
    endY=HIGH; 
  } 

    if (digitalRead(Z_STOP_PIN)== HIGH)
  {
    endZ=HIGH; 
  } 
}


/**
*  This function: set the pulse width with respect to the required speed in rmp
                  for the given axis 
*         Inputs:                  
*           W --> desire angular velociy in rmp 
*           axis --> select the axis to set the speed 
**/
void stepper_motion::set_Speed(float velocity, char axis, bool angular)
{
  float pulse_width,t_step, W=0;
 
  if (angular==true)
  {
      W= velocity;
  }
  else
  {
     W = linear_to_angular_velocity(velocity,  axis);
  }

   // saturation function to limit w between the margins of motor velocity
    if (W>w_max)
    {
       W=w_max;
    }
    else if (W< w_min)
    {
      W=0;
    }
    

  if (W!=0)
  {
  // each revolution is equal to 3200 steps for micro steps 
  // thus 'w* steps_per_revolution' is the numebr of steps for the given velocity to be passed in 60 minutes 
  // now compute the time for one step: 
    t_step= 60 / (W* steps_per_revolution); 
   
  // half of this time is used for low-step signal and the other half is used for high signal  (high level activation)
    pulse_width= (t_step) * e6; //convert the pulse width to micro unit 
  }

  else
  {
    pulse_width=0;
  }

// set the pulse width according to the selected axis
   switch (axis)
   {
   case 'x':
     PulseX = pulse_width;
     break;

   case 'y':
     PulseY = pulse_width;
     break;
     
   case 'z':
     PulseZ= pulse_width;
     break;
   
   default:
     break;
   }
}



/*
*  This function moves the extruder one step ahead with
*  the speed which should be set prior to this function
*  in  setfeedrate  
*  
*         
*/
void stepper_motion::move_extruder_onestep()
{
        digitalWrite(E0_STEP_PIN,LOW); 
        digitalWrite(E0_STEP_PIN,HIGH);
}


/**
* This function set a pulse width for the given  
* feed rate of the extrusion in mm/s 
*   Input:
*        V--> feedrate speed in mm/sec
*/
void stepper_motion::setFeedrate(float V)
{
    
  float w = V/R_extruder; // compute angular velocity 
  w = w * (30/PI); // convert to rmp

 if (w!=0)
  {
  // each revolution is equal to 3200 steps for micro steps 
  // thus 'w* steps_per_revolution' is the numebr of steps for the given velocity to be passed in 60 minutes 
  // now compute the time for one step: 
    float t_step= 60 / (w* steps_per_revolution); 
   
  // half of this time is used for low-step signal and the other half is used for high signal  (high level activation)
    PulseE= (t_step) * e6; //convert the pulse width to micro unit 
  }

  else
  {
    PulseE=0;
  }

   
}


/*
 This function moves the Z Axis with the set 
 velocity which should be determined before 
 calling this function by set_speed_RMP or
 set_speed_linear
   Inputs: 
         z-> distance to go in Z direction (mm)
         DIR-> direction to go 
   Output:
         moving z axis 

*/
void stepper_motion::move_axis_mm(float dist, char axis, int DIR)
{
  float teta=0; int stepM=0;
  
   switch (axis)
   {
   case 'x':
    teta = mm_to_deg(dist, 'x'); // convert mm to deg and compute number of steps requires for the motion 
    stepM = teta/step_angle;
    Direction_selection(DIR, 'x');
    for (int i = 0; i < stepM; i++)
    {
      Move_axis_onestep('x'); 
      delayMicroseconds(PulseX);
     }
     break;

   case 'y':
    teta = mm_to_deg(dist, 'y'); // convert mm to deg and compute number of steps requires for the motion 
    stepM = teta/step_angle;
    Direction_selection(DIR, 'y');
    for (int i = 0; i < stepM; i++)
    {
      Move_axis_onestep('y'); 
      delayMicroseconds(PulseY);
     }
     break;

   case 'z':
    teta = mm_to_deg(dist, 'z'); // convert mm to deg and compute number of steps requires for the motion 
    stepM = teta/step_angle;
    Direction_selection(DIR, 'z');
    for (int i = 0; i < stepM; i++)
    {
      Move_axis_onestep('z'); 
      delayMicroseconds(PulseZ);
     }
     break;

   case 'E':
    teta = mm_to_deg(dist, 'E'); // convert mm to deg and compute number of steps requires for the motion 
    stepM = teta/step_angle;
    Direction_selection(DIR, 'E');
    for (int i = 0; i < stepM; i++)
    {
      move_extruder_onestep(); 
      delayMicroseconds(PulseE);
     }
     break;

   default:
     break;
   }  

  

}






