// Feather I2c slave, radio master
/*
 * =======================================================================
 */
#include <Wire.h>

#define I2C_ADDR 0x08

void setup() 
{

  Serial.begin(19200);
  
  Wire.begin(I2C_ADDR);
  Wire.onReceive( receive_i2c_event );
  Wire.onRequest( request_i2c_event );

  // set up pins

  // set up radio :)

  // Wait untill serial is available to finish setup
  // theres no point it running if its not connected to a computer which is
  // using the serial.
  
  while( !Serial ) delay(1);
  
}

void loop() 
{
  // put your main code here, to run repeatedly:

}

void receive_i2c_event(int bytes)
{

}

void request_i2c_event()
{
  
}

void receive_radio_event(int bytes)
{

}

void request_radio_event()
{
  // debug only ? not needed ?
}
