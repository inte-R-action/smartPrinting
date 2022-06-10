
# include<Arduino.h>
//# include"ARYA/printer_motion.h"
//# include "ARYA/Fan.h"
//# include "ARYA/Display_LCD.h" 
//# include "ARYA/Thermistor.h"
//# include "ARYA/Tempreture.h"
//# include "ARYA/Planner.h"
//# include "ARYA/stepper_motion.h"
//# include "ARYA/Temp_control.h"
# include "ARYA/print_planning.h"


void draw_circle(int layer)
{
  float ext=0.3; // for 33.3 rmp
  for (int i = 0; i < layer; i++)
  {
    Move_XYaxis_simply_with_extrusion(-0.2462,3.1287 ,ext);
    Move_XYaxis_simply_with_extrusion(-0.7326,3.0517 ,ext);
    Move_XYaxis_simply_with_extrusion(-1.2010,2.8995 ,ext);
    Move_XYaxis_simply_with_extrusion(-1.6398,2.6759 ,ext);
    Move_XYaxis_simply_with_extrusion(-2.0382,2.3864 ,ext);
    Move_XYaxis_simply_with_extrusion(-2.3864,2.0382 ,ext);
    Move_XYaxis_simply_with_extrusion(-2.6759,1.6398 ,ext);
    Move_XYaxis_simply_with_extrusion(-2.8995,1.2010 ,ext);
    Move_XYaxis_simply_with_extrusion(-3.0517,0.7326 ,ext);
    Move_XYaxis_simply_with_extrusion(-3.1287,0.2462 ,ext);
    Move_XYaxis_simply_with_extrusion(-3.1287,-0.2462,ext);
    Move_XYaxis_simply_with_extrusion(-3.0517,-0.7326,ext);
    Move_XYaxis_simply_with_extrusion(-2.8995,-1.2010,ext);
    Move_XYaxis_simply_with_extrusion(-2.6759,-1.6398,ext);
    Move_XYaxis_simply_with_extrusion(-2.3864,-2.0382,ext);
    Move_XYaxis_simply_with_extrusion(-2.0382,-2.3864,ext);
    Move_XYaxis_simply_with_extrusion(-1.6398,-2.6759,ext);
    Move_XYaxis_simply_with_extrusion(-1.2010,-2.8995,ext);
    Move_XYaxis_simply_with_extrusion(-0.7326,-3.0517,ext);
    Move_XYaxis_simply_with_extrusion(-0.2462,-3.1287,ext);
    Move_XYaxis_simply_with_extrusion(0.2462 ,-3.1287,ext);    
    Move_XYaxis_simply_with_extrusion(0.7326 ,-3.0517,ext);    
    Move_XYaxis_simply_with_extrusion(1.2010 ,-2.8995,ext); 
    Move_XYaxis_simply_with_extrusion(1.6398 ,-2.6759,ext); 
    Move_XYaxis_simply_with_extrusion(2.0382 ,-2.3864,ext); 
    Move_XYaxis_simply_with_extrusion(2.3864 ,-2.0382,ext); 
    Move_XYaxis_simply_with_extrusion(2.6759 ,-1.6398,ext); 
    Move_XYaxis_simply_with_extrusion(2.8995 ,-1.2010,ext); 
    Move_XYaxis_simply_with_extrusion(3.0517 ,-0.7326,ext); 
    Move_XYaxis_simply_with_extrusion(3.1287 ,-0.2462,ext); 
    Move_XYaxis_simply_with_extrusion(3.1287 ,0.2462 ,ext); 
    Move_XYaxis_simply_with_extrusion(3.0517 ,0.7326 ,ext); 
    Move_XYaxis_simply_with_extrusion(2.8995 ,1.2010 ,ext); 
    Move_XYaxis_simply_with_extrusion(2.6759 ,1.6398 ,ext); 
    Move_XYaxis_simply_with_extrusion(2.3864 ,2.0382 ,ext);
    Move_XYaxis_simply_with_extrusion(2.0382 ,2.3864 ,ext);
    Move_XYaxis_simply_with_extrusion(1.6398 ,2.6759 ,ext);
    Move_XYaxis_simply_with_extrusion(1.2010 ,2.8995 ,ext);
    Move_XYaxis_simply_with_extrusion(0.7326 ,3.0517 ,ext);
    Move_XYaxis_simply_with_extrusion(0.2462 ,3.1287 ,ext);

    Ender3.move_axis_mm(0.3, 'z', Forward);
  }
  


}

void draw_controlled_line()
{
  float v_print =60; //mm/s
  float L_pass= 20; // mm
  float H_pass= 0.2; //mm
  float W_pass= 0.4; 

   Ender3.set_Speed(v_print,'x',false);
   Ender3.set_Speed(v_print,'y',false);
   Ender3.set_Speed(v_print,'z',false);

  // move to the middel of bed 
  Move_XYaxis_simply_with_extrusion(100,0,0);
  Move_XYaxis_simply_with_extrusion(0,100,0); 

  extrusion E;
  E= compute_extrusion_properties(L_pass, W_pass, H_pass, v_print);
  Ender3.setFeedrate(E.velocity);
  for (int  i = 0; i < 3; i++)
  {
      Move_XYaxis_simply_with_extrusion(0  , -L_pass, E.length);
      Move_XYaxis_simply_with_extrusion(-10,  L_pass, 0);
      Move_XYaxis_simply_with_extrusion(0  , L_pass, E.length);
  }
  Ender3.move_axis_mm(10, 'z',Forward);

}


void setup() 
{
  init_LCD();
  Serial.begin(9600);

  display_string("BioMed", 40, 32);
  delay(5000);

  print_init();
}



void loop()
{

   



  //draw_trapezoid(50);
  //draw_circle(50);
  //draw_raster(1, 3, 20);
  //Ender3.move_axis_mm(10, 'z',Forward);
  draw_controlled_line();
  while(1);
 


  /*
  // graphic commands to redraw the complete screen should be placed here  
  u8g.setFont(u8g_font_unifont);
  char HET[7],BT[7];
  dtostrf(read_extruder_tempreture(),7, 3, HET);
  dtostrf(read_Bed_tempreture(),7, 3, BT);
  
  u8g.firstPage();  
  do 
  {
    u8g.drawStr(0, 10, HET);
    u8g.drawStr(80, 32, BT);
   
  } while( u8g.nextPage() );

  */
  


 
 /*
   set_Fan_speed(200);
   printer_motion Ender3;
   float points[2][3]= {{0,0,0},{33.61*5,0,0}};
   float feed_rate = 0.5;
   float extruder_dist=1;
   Ender3.Trapezoid_planner(points, feed_rate, extruder_dist);
   set_Fan_speed(0);

   int hotend_status = Hotend_temprature_PID_control(65); 
   display_float(read_extruder_tempreture(),0,22);
   delay(500);
   
   while(1); 
   */
}
