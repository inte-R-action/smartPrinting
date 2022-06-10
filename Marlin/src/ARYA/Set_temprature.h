
/**
 * @file Set_temperature.h
 * @author Arya Shabani 
 * @brief this header file useses timer interrupts to:  A) raise the bed and and hotend 
 *        temperature to the predefined value and B) stabelized them to be around the 
 *        set value by applying PID within control timer
 * @version V 1.2 
 * @date 2022-04-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

# include "src/ARYA/Temperature.h"
#ifndef THERMISTOR_H
     # define THERMISTOR_H
     # include "src/ARYA/Thermistor.h"
#endif


// Allocate timer4 for monitoring tempreture interrupt 
TIM_TypeDef *inst1 = TIM4; // this a strucuter for timer variables (http://stm32.kosyak.info/doc/annotated.html)
HardwareTimer *monitor_timer = new HardwareTimer(inst1); // create instace of hardware class 

// Allocate timer5 for tempreture control interrupt 
TIM_TypeDef *inst2 = TIM5; // this a strucuter for timer variables (http://stm32.kosyak.info/doc/annotated.html)
HardwareTimer *control_timer = new HardwareTimer(inst2); // create instace of hardware class





#define PLA_hotend_temp 215
#define PLA_Bed_temp    65
#define temp_monitor_frequency 2 // HZ
#define temp_control_frequency 5 // HZ

bool stable_temp=false;
char counter=0;
bool control_bedtemp=false, control_endtemp=false;

/*
  This an interrupt function to monitor the 
  hotend empreture with temp_monitor_frequency
*/
void hotend_temp_monitor_callback()
{
  if (counter > 30)  
    {
      stable_temp=true;
      counter=0;
    }
     
  else if(abs(PLA_hotend_temp-read_extruder_tempreture()) < 1)
    {
      counter++;
    } 
}


/*
  This an interrupt function to monitor the 
  bed tempreture with temp_monitor_frequency
*/
void bed_temp_monitor_callback()
{
  if (counter > 30)  
    {
      stable_temp=true;
      counter=0;
    }
     
  else if(abs(PLA_Bed_temp-read_Bed_tempreture()) < 1)
    {
      counter++;
    } 
}

/*
  This an interrupt function that applies PID controller
  to the hotend basically and bed if its control flag within
  preheating_bed is actiavted
*/
void temp_control_callback()
{
  if (control_endtemp==true)
   {
     Hotend_temprature_PID_control(PLA_hotend_temp);
   }
  

  if (control_bedtemp==true)
   {
      Bed_temprature_PID_control(PLA_Bed_temp); 
   }
   
}



/* 
  This function runs a timer interrupt for monitoring 
  hotend temp  that measures the  stability of hotend temp 
  with temp_monitor_frequency thus after 30 times of finding 
  the tempeture within 1 dgree difference of desir temp it 
  stops monitoring  and transfer rest of procedure to keep
  the temp almost constant to the temp control interrupt 
*/
void preheating_hotend()
{
  stable_temp = false;
  control_endtemp=false;
  counter=0;

   // initialized the intrrupt for temp monitoring of hot end
   monitor_timer->setOverflow(temp_monitor_frequency, HERTZ_FORMAT);
   monitor_timer->attachInterrupt(hotend_temp_monitor_callback);
   monitor_timer->refresh();
   monitor_timer->resume(); 

 /*
    apply PID to hotend as long as the current temp is in 
    1 degree difference with desire temp for 30 times of 
    monitoring each 500 ms 
  */   while(!stable_temp)
   {  
     Hotend_temprature_PID_control(PLA_hotend_temp); 
   }

   // kill the monitoring interrupt
    monitor_timer->detachInterrupt();


  // if control interrupt is already called by preheating bed
  if(control_bedtemp==true)
  { 
    control_endtemp=true;
  }

  // if control interrupt hasn't called yet we call it here 
  else if(control_bedtemp == false)
  {
    control_endtemp=true;
    control_timer->setOverflow(temp_control_frequency,HERTZ_FORMAT);
    control_timer->attachInterrupt(temp_control_callback);
    control_timer->refresh();
    control_timer->resume();
  }


 
}


/* 
  This function runs a timer interrupt for monitoring 
  bed temp  that measures the  stability of bed temp 
  with temp_monitor_frequency thus after 30 times of finding 
  the tempeture within 1 dgree difference of desir temp it 
  stops monitoring  and transfer rest of procedure to keep
  the temp almost constant to the control interrupt that aleady
  ran in preheating_hot end  
*/
void preheating_bed()
{
  stable_temp = false;
  counter=0;
  control_bedtemp =false;

   // initialized the intrrupt for temp monitoring 
   monitor_timer->setOverflow(temp_monitor_frequency, HERTZ_FORMAT);
   monitor_timer->attachInterrupt(bed_temp_monitor_callback);
   monitor_timer->refresh();
   monitor_timer->resume(); 

 /*
    apply PID to hotend as long as the current temp is in 
    1 degree difference with desire temp for 30 times of 
    monitoring each 500 ms 
  */   while(!stable_temp)
   {  
       Bed_temprature_PID_control(PLA_Bed_temp); 
   }

   // kill the monitoring interrupt
    monitor_timer->detachInterrupt();
    
    // if control temp interrupt is already called by preheating_hotend()
    if (control_endtemp==true)
    {
       control_bedtemp=true;
    }
    
    // if control temp interrupt hasn't called yet we call it here 
    else if (control_endtemp==false)
    {
      control_bedtemp=true;
      control_timer->setOverflow(temp_control_frequency,HERTZ_FORMAT);
      control_timer->attachInterrupt(temp_control_callback);
      control_timer->refresh();
      control_timer->resume();
    }
}



