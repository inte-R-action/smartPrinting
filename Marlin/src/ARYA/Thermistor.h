/* 
   This header file reades temperatur of the hotend and bed utilizing the termistor:
      to read the themistor resistance we install a pull up resistor in serie with thermistor {in ender3 R_pullup: 4.7 KOHM} 
      according to the vlotage low the mount of voltage on the pull up resistor(that we read by arduino) is equal to: 
                    Vo = R/(R+4700)*Vcc ==> R= 4700 /((Vcc/Vo)-1)
                    Vcc is the maximum availabe voltage: 1023  
      On the other hand the temprature can be computed base on the thermistor resistance by following formula: 
          1/T = (1/T0) + (1/B) *ln(R/R0) w
          where T is current temp in Kelvin, 
          T0(298.15) is room temp in Kelvin, 
          R0(100000) is resistance in room temp 
          R in current resistance 
          B is Beta coefficient for resistance tempreture curve 
   we define: C1=(1/T0), C2:(1/B) 
                       T= 1/ ( C1 + C2_HOTEND*ln(R/R0) )
** in order to remove noises thermiostor value is averaged over some cycles
*/



#ifndef Creality_V422_pins_h
   #define Creality_V422_pins_h
   #include "Creality_V422_pins.h"
#endif


#define HOTEND_PULLUP_RESISTOR_OHMS  4700     // Pullup resistor
#define HOTEND_RESISTANCE_25C_OHMS  100000  // Resistance at 25C
#define HOTEND_BETA                 3950   // Beta value
#define BED_PULLUP_RESISTOR_OHMS     4700    // Pullup resistor
#define BED_RESISTANCE_25C_OHMS      100000  // Resistance at 25C
#define BED_BETA                     3950    // Beta value

#define cycle_to_read_thermistor     4
#define C1          0.00335401643468053   // (1/T0)
#define C2_HOTEND   0.000253164556962025 //(1/Beta)
#define C2_BED      0.000253164556962025   // (1/Beta)
#pragma once 




/*
* This function: 
  measure the temperature of the hotend 
*/
double read_extruder_tempreture(void)
{
   double R_termistor=0, v_read=0;
 

 for (int i = 0; i < cycle_to_read_thermistor; i++)
 {
    v_read=analogRead(TEMP_0_PIN);
    R_termistor += HOTEND_PULLUP_RESISTOR_OHMS /  ( (1023.0 / v_read) - 1.0); 
 }
   R_termistor= R_termistor/cycle_to_read_thermistor;


   double Temp = 1/ ( C1 + C2_HOTEND* log(R_termistor/HOTEND_RESISTANCE_25C_OHMS) );
   Temp = Temp - 273.15;

   return Temp;
}




/*
* This function: 
  measure the temperature of the bed 
*/
double read_Bed_tempreture(void)
{
  double R_termistor=0, v_read=0;
 

 for (int i = 0; i < cycle_to_read_thermistor; i++)
 {
    v_read=analogRead(TEMP_BED_PIN);
    R_termistor += BED_PULLUP_RESISTOR_OHMS /  ( (1023.0 / v_read) - 1.0); 
 }
   R_termistor= R_termistor/cycle_to_read_thermistor;


 double Temp = 1/ ( C1 + C2_BED* log(R_termistor/BED_RESISTANCE_25C_OHMS) );
 Temp = Temp - 273.15;

   return Temp;
}