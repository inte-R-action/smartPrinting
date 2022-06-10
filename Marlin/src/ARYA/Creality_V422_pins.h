#include <pins_arduino.h>
#pragma once

/**
 * Creality 4.2.x (STM32F103RET6) board pin assignments
 */





//
// Limit Switches
//
#define X_STOP_PIN                          PA5
#define Y_STOP_PIN                          PA6
#define Z_STOP_PIN                          PA7


//
// Steppers
//
#ifndef X_STEP_PIN
  #define X_STEP_PIN                        PC2
#endif
#ifndef X_DIR_PIN
  #define X_DIR_PIN                         PB9
#endif
#define X_ENABLE_PIN                        PC3   // Shared

#ifndef Y_STEP_PIN
  #define Y_STEP_PIN                        PB8
#endif
#ifndef Y_DIR_PIN
  #define Y_DIR_PIN                         PB7
#endif
#define Y_ENABLE_PIN                X_ENABLE_PIN

#ifndef Z_STEP_PIN
  #define Z_STEP_PIN                        PB6
#endif
#ifndef Z_DIR_PIN
  #define Z_DIR_PIN                         PB5
#endif
#define Z_ENABLE_PIN                X_ENABLE_PIN

#ifndef E0_STEP_PIN
  #define E0_STEP_PIN                       PB4
#endif
#ifndef E0_DIR_PIN
  #define E0_DIR_PIN                        PB3
#endif
#define E0_ENABLE_PIN               X_ENABLE_PIN


// RET6 12864 LCD + encoder 
#define LCD_PINS_RS                     PB12
#define LCD_PINS_ENABLE                 PB15
#define LCD_PINS_D4                     PB13
 
#define BTN_ENC                         PB2
#define BTN_EN1                         PB10
#define BTN_EN2                         PB14

//
// Heaters / Fans
//
#define HEATER_PIN                        PA1   // hotend
#define HEATER_BED_PIN                    PA2   // HOT BED
#define FAN_PIN                           PA0   // FAN

  

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC5   // TH1
#define TEMP_BED_PIN                        PC4   // TB1