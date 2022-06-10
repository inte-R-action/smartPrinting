#include "LCD_lib/src/U8glib.h"


#ifndef Creality_V422_pins_h
  #define Creality_V422_pins_h
   #include "Creality_V422_pins.h"
#endif

# ifndef Arduio_h
   #define Arduino_h
   #include <Arduino.h>
# endif

#pragma once

U8GLIB_ST7920_128X64_1X u8g(PB13, PB15, PB12 );	// SPI Com: [D4,EN,RS]


void init_LCD()
{
     // flip screen, if required
  // u8g.setRot180();
  
  // set SPI backup if required
  //u8g.setHardwareBackup(u8g_backup_avr_spi);

  // assign default color value
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
  
}


void draw_float_as_string(float number, unsigned int X_cursor, unsigned int Y_cursor ) {
  // graphic commands to redraw the complete screen should be placed here  
  u8g.setFont(u8g_font_unifont);
  char outstr[15];
  dtostrf(number,7, 3, outstr);
  u8g.drawStr(X_cursor, Y_cursor, outstr);
}


void display_float(float number, unsigned int X_cursor, unsigned int Y_cursor )
{
  u8g.firstPage();  
  
  do {
    draw_float_as_string(number, X_cursor, Y_cursor);
  } while( u8g.nextPage() );

}

void display_string(char str[], unsigned int X_cursor, unsigned int Y_cursor)
{
  // graphic commands to redraw the complete screen should be placed here  
  u8g.setFont(u8g_font_unifont);
 //u8g.setFont(u8g_font_osb21);
  u8g.firstPage();  
  do {
    u8g.drawStr( X_cursor, Y_cursor, str);
  } while( u8g.nextPage() );

}


void monitor_page(float hotend_temp, float bed_temp, int x_position, int y_position )
{

  char X[3], Y[3], Te[3], Tb[3];
  itoa(x_position, X, 10);
  itoa(y_position, Y, 10);
  itoa(int(hotend_temp), Te, 10);
  itoa(int(bed_temp), Tb, 10);

  // graphic commands to redraw the complete screen should be placed here  
  u8g.setFont(u8g_font_5x7);
  u8g.firstPage();  
  do {
    u8g.drawStr( 5, 15, "positions:");
    u8g.drawStr( 5, 30, "X:");
    u8g.drawStr( 15, 30, X);
    u8g.drawStr( 35,30, "Y:");
    u8g.drawStr( 45, 30, Y);


    u8g.drawStr( 70, 35, "Temp:");
    u8g.drawStr( 55, 50, "bed:");
    u8g.drawStr( 75, 50, Tb);
    u8g.drawStr( 95,50, "end:");
    u8g.drawStr( 115, 50, Te);
  } while( u8g.nextPage() );

}