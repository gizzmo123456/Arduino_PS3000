// Feather I2c slave, radio master
/*
 * =======================================================================
 * prints '1' to serial once setup is compleat.
 * Serial must be available to compleat setup :)
 */
#include <Wire.h>
#include <rfm_setup.h>
#include <RH_RF69.h>
#include <MPU6050_tockn.h>

#define I2C_ADDR      0x08
#define MUX_ADDR      0x70

#define MOTOR_L_VALUE 1
#define MOTOR_C_VALUE 2
#define MOTOR_R_VALUE 4

#define MOTOR_ACTIVE_TIME 10000   //ms (20sec)
#define TOTAL_MOTORS 3

#define RFM_CS  8
#define RFM_INT 7
#define RFM_RES 4

#define OUTPUT_BUFFER_SIZE  6

RH_RF69 rfm(RFM_CS, RFM_INT);
String radio_message = "";

// todo: send motor message.
int motor_values[] {MOTOR_L_VALUE, MOTOR_C_VALUE, MOTOR_R_VALUE};
float motor_end_times[] {0, 0, 0};
int8_t motor_output_value = 0;

// Gyros I2c
String i2c_message = "";
MPU6050 MPU[3](Wire);

// debuging
bool debug_mode = false;
bool debug_serial_output = false;
bool debug_no_serial_output = false;

String GetPaddedString(int num)
{
  //define buffer and the final padded values
  char buff[ OUTPUT_BUFFER_SIZE ];
  char padded[ OUTPUT_BUFFER_SIZE + 1 ];
  
  bool neg = false; //remember if num is negitive

  if(num < 0)       // make shore that the number is positive since we are converting unsigned int to char
  {
    num = -num;
    neg = true;
  }
  
  sprintf(buff, "%.5u", num); //Convert num to chars

  // padd the buffer 
  for(int i = 0; i < OUTPUT_BUFFER_SIZE; i++)
    padded[i] = buff[i];

  padded[OUTPUT_BUFFER_SIZE] = '\0';
  if(neg) padded[0] = '-';  

  String paddedStr = String(padded);

  if (debug_mode)
  {
    Serial.print( String(padded) ); // Output the padded value to serial console :)
    Serial.print("#");
  }

  return paddedStr;
  
}

void MUX_select( int8_t i2c_bus)
{

  if( i2c_bus < 0 || i2c_bus > 7 ) return;

  Wire.beginTransmission( MUX_ADDR ); 
  Wire.write( 1 << i2c_bus );
  Wire.endTransmission(true);
  
}

void setup() 
{

  Serial.begin(19200);

  // Set up I2c slave :)
  Wire.begin(I2C_ADDR);
  Wire.onReceive( receive_i2c_event );
  Wire.onRequest( request_i2c_event );

  // set up pins
  pinMode(RFM_RES, OUTPUT);
  digitalWrite(RFM_RES, LOW);
  rfm_reset();
  
  // set up radio :)
  rfm_setup(&rfm);

  // set up the 3 mpu's
  for(int i = 0; i < 3; i++)
    MPU[i].begin();
  
  // Wait untill serial is available to finish setup
  // theres no point it running if its not connected to a computer which is
  // using the serial.
  while( !Serial ) delay(1);
  Serial.println(1);
  
}

void loop() 
{
  // Update the 3 MPU's
  MUX_select(0);
  MPU[0].update();
  MUX_select(1);
  MPU[1].update();
  MUX_select(2);
  MPU[2].update();
    
  read_serial();
  
  motor_output_value = get_motor_output_value();
  receive_radio_event();
  
  if( debug_serial_output )
    write_serial();

  if ( debug_mode )
    Serial.println("==========================");
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
    else if( !debug_serial_output && (incoming_byte == 'I' || incoming_byte == 'i' ) )
    { // if debug_serial_output then write serial is called from loop
      write_serial();
    }
    else if( incoming_byte == 'n' || incoming_byte == 'N')
    { // Normalize MPU's
      for(int i = 0; i < 3; i++)
        MPU[i].normalize();

      request_radio_event('N');
        
    }
    else if( incoming_byte == 'd' || incoming_byte == 'D' )
    {
      debug_mode = incoming_byte == 'D';
    }
    else if( incoming_byte == 's' || incoming_byte == 'S' )
    {
      debug_serial_output = incoming_byte == 'S';
    }
    else if( incoming_byte == 'E' || incoming_byte == 'e' )
    {
      debug_no_serial_output = incoming_byte == 'E';
    }
  }
  
}

void write_serial()
{

  // This is a lil confussing so here goes.
  // since i want to keep the orginal formating.
  // Serial output format: Gyro_0_x gyro_0_y hobDist_0 hob_0 [0, 1, 2]
  //                       jug_gryo_x jug_gryo_y jug_gryo_z whisking
  // we get the gyro directly from the devices.
  // the hobDist and hob come in over i2c from the UNO
  // format: hobDist_0 hob_0 [0, 1, 2] (each value len 5)
  // the jug gryo and whisking coming via radio.
  // format: jug_gryo_x jug_gryo_y jug_gryo_z whisking (each value len 5)

  if( debug_no_serial_output ) return;

  String serial_output = "";

  // print the 3 pan/hob values
  for(int i = 0; i < 3; i++)
  {
    serial_output += get_mpu_string(i);
    
    if(debug_mode || debug_serial_output)
      serial_output += "|";
      
    serial_output += i2c_message.substring(10*i, 10*(i+1));
    
    if(debug_mode || debug_serial_output)
      serial_output += "|";
  }

  // print jug values and whisking
  serial_output += radio_message;

  if( debug_mode )
    Serial.print(" Serial Message: ");
    
  Serial.println(serial_output);

}

String get_mpu_string(int8_t mpu_id)
{

  String str = GetPaddedString( (MPU[mpu_id].getAngleX( true )) );
  str += GetPaddedString( (MPU[mpu_id].getAngleY( true )) );

  return str;

}

void receive_i2c_event( int bytes )
{

  char message[bytes + 2];
  int current_byte = 0;

  while( Wire.available() )
  {
    message[current_byte] = Wire.read();
    current_byte++;
  }

  message[current_byte] = '\0';
  i2c_message = message;

  if ( debug_mode )
  {
    Serial.print("i2c message recived: ");
    Serial.println(message);
  }

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

  if( debug_mode )
    Serial.println( motor_output_value );
}

void receive_radio_event()
{
  
  if( rfm.available() )
  {

    uint8_t buff[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buff);

    if( debug_mode )
      Serial.println("--Reciving message over radio--");
    
    if( rfm.recv(buff, &len) )
    {
      //if (!len) return;
      buff[len] = 0;
      
      // store data.
      radio_message = (char *)buff;
      if( debug_mode ) 
      {
        Serial.println( (char*)buff );
        Serial.print( "Length: " );
        Serial.println( len );
      }
      
    }    
  }
  
}

void request_radio_event(char e)
{
  // debug only ? not needed ?
  char message[2] = "";
  message[0] = e;
  message[1] = "\0";
  
  rfm.send( (uint8_t*) message, strlen(message) );
  rfm.waitPacketSent();

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

void rfm_reset()
{
  digitalWrite(RFM_RES, HIGH);
  delay(10);
  digitalWrite(RFM_RES, LOW);
  delay(10);
}
