// Feather I2c slave, radio master
/*
 * =======================================================================
 */
#include <Wire.h>
#include <rfm_setup.h>
#include <RH_RF69.h>

#define I2C_ADDR      0x08

#define MOTOR_L_VALUE 1
#define MOTOR_C_VALUE 2
#define MOTOR_R_VALUE 4

#define MOTOR_ACTIVE_TIME 2000   //ms (20sec)
#define TOTAL_MOTORS 3

// todo: send motor message.
int motor_values[] {MOTOR_L_VALUE, MOTOR_C_VALUE, MOTOR_R_VALUE};
float motor_end_times[] {0, 0, 0};
int8_t motor_output_value = 0;


bool debug_mode = false;

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
  Serial.println(1);
  
}

void loop() 
{
  // put your main code here, to run repeatedly:

  read_serial();
  
  motor_output_value = get_motor_output_value();
  
  write_serial();
  
}

void read_serial()
{
  // read all incoming bytes
  while( Serial.available() > 0 )
  {
    char incoming_byte = Serial.read();
    // A = left motor on, B = center motor, C = right motor
    // 'A' is 65 in Ascii
    if( incoming_byte - 65 >= 0 && incoming_byte - 65 < 3 )
    {
      int motor_id = incoming_byte - 65;  //motors 0, 1, 2
      set_motor_active( motor_id );
    }
    else if( incoming_byte == 'd' || incoming_byte == 'D' )
    {
      debug_mode = incoming_byte == 'd';
    }
  }
  
}

void write_serial()
{
  
}

void receive_i2c_event(int bytes)
{

}

void request_i2c_event()
{
  if( debug_mode )
  {
    Serial.println("--Sending message over i2c--");
    Serial.println( motor_output_value );
  }
  // send output
  Wire.write( motor_output_value );
}

void receive_radio_event(int bytes)
{

}

void request_radio_event()
{
  // debug only ? not needed ?
}

void set_motor_active( int mid )
{
  motor_end_times[mid] = millis() +  MOTOR_ACTIVE_TIME;
}

int get_motor_output_value()
{
  int output_value = 0;
  
  for ( int i = 0; i < TOTAL_MOTORS; i++ )
    if( millis() < motor_end_times[ i ] )
      output_value += motor_values[ i ];

  return output_value;
}
