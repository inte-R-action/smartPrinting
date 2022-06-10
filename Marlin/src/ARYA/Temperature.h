/**
 * @file Temperature.h
 * @author Arya Shabani 
 * @brief This headr file applys a PID controller to temperature of 
 *        bed and hotend with PID coefficients that has been detected 
 *        by Marline frameware 
 * @version V 1.2 
 * @date 2022-04-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef ARDUINO_H
     # define ARDUINO_H
     #include <Arduino.h>
#endif

#ifndef LCD_H
     # define LCD_H
     # include "src/ARYA/Display_LCD.h"
#endif

#ifndef THERMISTOR_H
     # define THERMISTOR_H
     # include "src/ARYA/Thermistor.h"
#endif

# include "src/ARYA/PID_lib/PID.h"



// Below this temperature the heater will be switched off
// because it probably indicates a broken thermistor wire.
#define HEATER_MINTEMP   0

// Above this temperature the heater will be switched off.
// This can protect components from overheating, but NOT from shorts and failures.
// (Use MINTEMP for thermistor short/failure protection.)
#define HEATER_MAXTEMP 275

/**
 * Thermal Overshoot
 * During heatup (and printing) the temperature can often "overshoot" the target by many degrees
 * (especially before PID tuning). Setting the target temperature too close to MAXTEMP guarantees
 * a MAXTEMP shutdown! Use these values to forbid temperatures being set too close to MAXTEMP.
 */
#define HOTEND_OVERSHOOT 15   // (°C) Forbid temperatures over MAXTEMP - OVERSHOOT
#define BED_OVERSHOOT    10   // (°C) Forbid temperatures over MAXTEMP - OVERSHOO

// PID coefficients of hotend and Bed
#define DEFAULT_hotendKp  28.72
#define DEFAULT_hotendKi   2.62
#define DEFAULT_hotendKd  78.81
#define DEFAULT_bedKp 462.10
#define DEFAULT_bedKi 85.47
#define DEFAULT_bedKd 624.59

#pragma once

// variable defition for PID controller (current_temp , control_signal, desired_temp)
double HOTEND_temp, HOTEND_DUTY_CYCLE, HOTEND_SETPOINT;
double BED_temp, BED_DUTY_CYCLE, BED_SETPOINT;

// build intances of PID class for hotend and bed tempreture control 
PID hotend_pid (&HOTEND_temp, &HOTEND_DUTY_CYCLE, &HOTEND_SETPOINT, DEFAULT_hotendKp, DEFAULT_hotendKi,DEFAULT_hotendKd, DIRECT);
PID bed_pid (&BED_temp, &BED_DUTY_CYCLE, &BED_SETPOINT, DEFAULT_bedKp, DEFAULT_bedKi,DEFAULT_bedKd, DIRECT);


/*
 set PID for hotend with given coefficients 
*/
void setup_hotend_PID(void)
{
    hotend_pid.SetMode(AUTOMATIC);
    hotend_pid.SetTunings(DEFAULT_hotendKp, DEFAULT_hotendKi, DEFAULT_hotendKd);
}


/*
* This function apply PID control for ONE-STEP on duty cycle of output pin
* to controll tempreture of the hotend.
* (this function should be in loop to guarantee contineous control of tempreture) 
*  this function also take care of over heating and not-functioning thermistor 
*     Inputs: temp--> desire tempreture 
*     Output: -1--> thermistor problem->0 PID 
*             -2--> overheating problem->0 PID 
*              1--> normal 
*/ 
int  Hotend_temprature_PID_control(float temp)
{
   HOTEND_SETPOINT=temp;
   setup_hotend_PID();

   HOTEND_temp = read_extruder_tempreture();
   
   // probably the thermistor is not working (turn off!)
   if (HOTEND_temp == HEATER_MINTEMP)
   {
      analogWrite(HEATER_PIN, 0); 
      display_string("temp_broken", 5, 22);
      return -1;
   }

   // the temperature is too close to the Maximum (turn off!)
   else if(HOTEND_temp >= (HEATER_MAXTEMP-HOTEND_OVERSHOOT))
   {
      analogWrite(HEATER_PIN, 0); 
      display_string("over_heat", 5, 22);
      return -2;
   }
   
   else{
         hotend_pid.Compute();
         analogWrite(HEATER_PIN,HOTEND_DUTY_CYCLE); 
         return 1;
       }
}





/*
 set PID for bed with given coefficients 
*/
void setup_bed_PID(void)
{
    bed_pid.SetMode(AUTOMATIC);
    bed_pid.SetTunings(DEFAULT_bedKp, DEFAULT_bedKi, DEFAULT_bedKd);
}


/*
* This function apply PID control for ONE-STEP on duty cycle of output pin
* to controll tempreture of the bed.
* (this function should be in loop to guarantee contineous control of tempreture) 
*  Exeptions::: over-heating and not-functioning thermistor 
*     Inputs: temp--> desire tempreture 
*     Output: -1--> thermistor problem-> 0 PID 
*             -2--> overheating problem-> 0 PID
*              1--> normal 
*/
int  Bed_temprature_PID_control(float temp)
{
   BED_SETPOINT=temp;
   setup_bed_PID();

   BED_temp = read_Bed_tempreture();
   
   // probably the thermistor is not working (turn off!)
   if (BED_temp == HEATER_MINTEMP)
   {
      analogWrite(HEATER_BED_PIN, 0); 
      display_string("temp_broken", 5, 22);
      return -1;
   }

   // the temperature is too close to the Maximum (turn off!)
   else if(BED_temp >= (HEATER_MAXTEMP-BED_OVERSHOOT))
   {
      analogWrite(HEATER_BED_PIN, 0); 
      display_string("over_heat", 5, 22);
      return -2;
   }
   
   else{
         bed_pid.Compute();
         analogWrite(HEATER_BED_PIN,BED_DUTY_CYCLE); 
         return 1;
       }
}