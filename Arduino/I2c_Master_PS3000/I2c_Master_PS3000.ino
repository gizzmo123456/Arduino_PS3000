// Arduino UNO Master I2c.
/* =======================================================================
 * Use the Arduino Uno as the master to contiunelesly send data to
 * a feather 32u4 RFMx (salve I2c) over the I2c.  
 * The Feather Board is used to commuacate with a PC over Serial.
 * This config has be used sine the feather is a faster board than the UNO
 * so we prevent any hanging/waiting.
 * Also because the feather also recives data over a radio from another
 * feather 32u4 RFMx. making it the central point between the 3 PCB's
 * =======================================================================
 * Moreover, The UNO is used for the direct visual & audio feedback to 
 * the player via a prezo speaker and 3 motors which blow ribon both as a 
 * means of fire feedback.
 * Also the UNO is responable for LDR inputs (pan on hob) 
 * and potentiometers inputs (temperture)
 * Altho the one of the set of inputs might get moved to the feather
 * We'll see.
 * =======================================================================
 * To detect if feedback should be On pins 2, 3, 4 will be HIGH
 * pin 2 is LHS motor (left to right) if any pins are HIGH the prezo
 * will be active :) (this might change so it is on none PWM pins)
 */

#include <Wire.h>

// Fire Feedback  
// The motor inputs can be replaced with by mesuring the paulse timeings
// from a pwm pin. But this causes a lil bit of hanging around when the 
// pin is low. But if we run out of pins it is an option :)
#define MOTOR_L_INPUT  2 // Left     //HIGH is ACTIVE
#define MOTOR_C_INPUT  3 // Center   //HIGH is ACTIVE
#define MOTOR_R_INPUT  4 // Right    //HIGH is ACTIVE

#define MOTOR_L_OUTPUT  9
#define MOTOR_C_OUTPUT  10
#define MOTOR_R_OUTPUT  11

#define PIEZO_OUTPUT    6

// Inputs
#define LDR_L_INPUT     A3
#define LDR_C_INPUT     A4
#define LDR_R_INPUT     A5

#define POTENT_L_INPUT  A0
#define POTENT_C_INPUT  A1
#define POTENT_R_INPUT  A2

// I2c
#define I2C_SLAVE       0x08

void setup() {

  Serial.begin(19200);    // used for debugs.
  Wire.begin();           // make this device our master

  //Set up pins.
  pinMode(MOTOR_L_INPUT, INPUT);
  pinMode(MOTOR_C_INPUT, INPUT);
  pinMode(MOTOR_R_INPUT, INPUT);

  pinMode(MOTOR_L_OUTPUT, OUTPUT);
  pinMode(MOTOR_C_OUTPUT, OUTPUT);
  pinMode(MOTOR_R_OUTPUT, OUTPUT);

  pinMode(LDR_L_INPUT, INPUT);
  pinMode(LDR_C_INPUT, INPUT);
  pinMode(LDR_R_INPUT, INPUT);

  pinMode(POTENT_L_INPUT, INPUT);
  pinMode(POTENT_C_INPUT, INPUT);
  pinMode(POTENT_R_INPUT, INPUT);
  
}

void loop() 
{

  // Last thing send the data over the i2c :D

}
