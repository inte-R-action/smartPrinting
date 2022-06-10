#ifndef Creality_V422_pins_h
   #define Creality_V422_pins_h
   #include "Creality_V422_pins.h"
#endif

#ifndef ARDUINO_H
     # define ARDUINO_H
     #include <Arduino.h>
#endif


#define e3 1000
#define e6 1000000 
#define Forward 1
#define Backward -1


#pragma once


class stepper_motion
{
  private:
     // set structural charastristics of the Ender3 
      const float R_pulley = 6.36; // pulley radius in (mm) (Ender 3) 
      const unsigned int   teeth =20; //  number of gear theet on x and y pulley in Ender3 
      const float P_pulley= 2; //(R_pulley / teeth)*2*PI;  pulley pitch (1.68 mm)
      const unsigned int P_screw = 8; // screw_pitch (mm)
      const unsigned int R_extruder =5; // radius of the extruder gear (mm) 
      const unsigned int w_max = 440; // maximum allowed angular velocity (heuristically)[rpm]
      const unsigned int w_min = 0.1; // minimum allowed angular velocity (heuristically)[rmp]
      const unsigned int V_max = 100; // maximum velocity for trapezoid planner
      const unsigned int a_max =500; // maximum acceleration for trapezoid planner
      int DIRx=0, DIRy=0, DIRz=0;

      


  public:
  const float step_angle = 1.8/16;
  const unsigned int steps_per_revolution= 3200;
  byte endX= LOW, endY= LOW, endZ= LOW; // these flags determines that the axis reaches to its endpoint(high end, low normal)  
  float  PulseE=0, PulseX=0, PulseY=0, PulseZ=0;// define pulse width for extruder, X,Y,Z axes
 
  stepper_motion();
  void  init();
  float mm_to_deg(float D, char axis);
  float linear_to_angular_velocity(float V, char axis);
  void Direction_selection(int DIR, char axis);
  void Move_axis_onestep(char axis);
  void set_Speed(float velocity, char axis, bool angular);
  void MonitorMicroSwitches();
  void setFeedrate(float V);
  void move_extruder_onestep();
  void move_axis_mm(float dist, char axis, int DIR);

};

