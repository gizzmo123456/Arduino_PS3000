// Arduino UNO I2C Master.
/* =======================================================================
 * Use the Arduino Uno as the master to contiunelesly send data to
 * a feather 32u4 RFMx over the I2c (slave).  
 * The Feather Board is used to commuacate with a PC over Serial.
 * This config has be used since the feather is a faster board than the UNO
 * so we prevent any hanging/waiting.
 * Also because the feather also recives data over a radio from another
 * feather 32u4 RFMx. making it the central point between the 3 PCB's
 * =======================================================================
 * Moreover, The UNO is used for the direct visual & audio feedback to 
 * the player via a prezo speaker and 3 motors which blows ribon both as a 
 * means of fire feedback.
 * Also the UNO is responable for LDR inputs (pan on hob) 
 * and potentiometers inputs (temperture)
 * Altho the one of the set of inputs might get moved to the feather
 * We'll see.
 * =======================================================================
 * The Uno will request if any feedback should be applied every 200ms 
 * (atm, see #define MOTOR_UPDATE_INTERVALS) over I2c
 * motor L is 1, motor C is 2 and motor R is 4. theres numbers are added
 * together and sent to the uno. if the number is 0 then they are all off
 */

#include <Wire.h>

// Fire Feedback  
// The motor inputs can be replaced with by mesuring the paulse timeings
// from a pwm pin. But this causes a lil bit of hanging around when the 
// pin is low. But if we run out of pins it is an option :)
// TODO:
// insted of the motors each having an input and an out pin, it should be requested over
// i2c every ~0.2secs and have the value cleared on the feather and and timed
// on the Arduino UNO.  

// TODO: sort out the comments above.
#define MOTOR_UPDATE_INTERVALS  200

#define MOTOR_L_OUTPUT    9
#define MOTOR_C_OUTPUT    10
#define MOTOR_R_OUTPUT    11
#define MOTOR_REV_OUTPUT  12  // THIS PIN MUST ALWAYS BE LOW, otherwise the motors will run in reverse

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

const uint8_t TOTAL_INPUTS = 3;

// inputs (0 = Left, 1 = Center, 2 = Right)
uint16_t ldr[]    {0, 0, 0};
uint16_t potent[] {0, 0, 0};

// outputs (0 = Left, 1 = Center, 2 = Right)
bool motor_active[]  {0, 0, 0};
unsigned long motor_next_update = MOTOR_UPDATE_INTERVALS;
bool fire_alarm_is_active = false;

// Debug
bool debug_serial = false;

void setup() {

  Serial.begin(19200);    // used for debuging only via usb.
  Wire.begin();           // make this device our master

  //Set up pins.
  pinMode(MOTOR_L_INPUT, INPUT);
  pinMode(MOTOR_C_INPUT, INPUT);
  pinMode(MOTOR_R_INPUT, INPUT);

  pinMode(MOTOR_L_OUTPUT, OUTPUT);
  pinMode(MOTOR_C_OUTPUT, OUTPUT);
  pinMode(MOTOR_R_OUTPUT, OUTPUT);
  pinMode(MOTOR_REV_OUTPUT, OUTPUT);

  pinMode(PIEZO_OUTPUT, OUTPUT);
  
  pinMode(LDR_L_INPUT, INPUT);
  pinMode(LDR_C_INPUT, INPUT);
  pinMode(LDR_R_INPUT, INPUT);

  pinMode(POTENT_L_INPUT, INPUT);
  pinMode(POTENT_C_INPUT, INPUT);
  pinMode(POTENT_R_INPUT, INPUT);

  // set all the outputs to LOW
  // to insure that all the outputs are not active when the controller starts :D
  digitalWrite(PIEZO_OUT, LOW);
  
  digitalWrite(MOTOR_L_OUTPUT, LOW);
  digitalWrite(MOTOR_C_OUTPUT, LOW);
  digitalWrite(MOTOR_R_OUTPUT, LOW);
  
  digitalWrite(MOTOR_REV_OUTPUT, LOW);
  
}

void loop() 
{

  read_inputs();
  update_outputs();
  send_message_to_slave();
  request_data_from_slave();  // todo add method to be able to disable this. if another ardiuno is not connented
  serial_debug();
  
}

void read_inputs()
{
  // LDR

  // Potent
  
}

void update_outputs()
{

  // update fire feedback outputs
  for( int i = 0; i < 3; i++)
    digitalWrite(PIEZO_OUTPUT, motor_active[i]);
  
  // if any fire sound alarm
  digitalWrite(PIEZO_OUTPUT, fire_alarm_is_active);
  
}

void send_message_to_slave()
{
  
}

void request_data_from_slave()
{

  if ( millis() > motor_next_update ) return;

  Wire.requestFrom( I2C_SLAVE, 1 );

  int incoming_byte;

  while( Wire.available() )
  {
    incoming_byte = Wire.read();
  }

  motor_next_update = millis() + MOTOR_UPDATE_INTERVALS;
  
}

void set_motors_active( int value )
{ // needs testing

  int max_motor_value = 4;
  int motor_id = 2;
  bool sound_alarm = false;
  
  while ( motor_id >= 0 )
  {
    
    if ( max_motor_value <= value )
    {
      value -= max_motor_value;
      motor_active[ motor_id ] = true;
      sound_alarm = true;
    }
    else
    {
      motor_active[ motor_id ] = false;
    }

    max_motor_value = max_motor_value >> 1;
    motor_id--;

  }

  fire_alarm_is_active = sound_alarm;

}

// debug mode can only be accesed via USB serial.
// send 'D' to turn serial debug on and 'd' to turn serial debug off. 
void serial_debug()
{

  if( Serial.available() > 0)
  {
    byte incoming_byte = Serial.read();
    if ( incoming_byte == 'd' || incoming_byte == 'D' )
      debug_serial = incoming_byte == 'D';
  }
  
  if( !debug_serial ) return;  
}
