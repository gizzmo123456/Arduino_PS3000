// Sends jug gyro and whih value over radio :)

#include <Wire.h>
#include <rfm_setup.h>
#include <RH_RF69.h>
#include <MPU6050_tockn.h>

#define OUTPUT_BUFFER_SIZE  6

#define RFM_CS    8
#define RFM_INT   7
#define RFM_RES   4

#define WHISK_INPUT   A0

RH_RF69 rfm(RFM_CS, RFM_INT);

MPU6050 MPU(Wire);
char padded[ OUTPUT_BUFFER_SIZE + 1 ];

const int whisking_switch_lowValue = 100;
int whisking_lastWasLow = false;
const int whisking_valueChanged_interval = 250; // ms
unsigned long whisking_nextInterval = 200;

int whiskValue = 0;
bool whisking = false;

// debuging
bool debug_serial = false;

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
/*
  if (debug_serial)
  {
    Serial.print( String(padded) ); // Output the padded value to serial console :)
    Serial.print("#");
  }
*/
  return padded;//paddedStr;
  
}

void setup() 
{

  Serial.begin(19200);

  Wire.begin();

  pinMode(RFM_RES, OUTPUT);
  digitalWrite(RFM_RES, LOW);
  rfm_reset();

  rfm_setup(&rfm);

  MPU.begin();

  Serial.println("Start");

}

void loop()
{
  MPU.update();
  UpdateWhisking();
  
  send_radio_event();
  receive_radio_event();
  
  debuging();
  
}

// Send message over radio
// Format: mpu.x mpu.y mpy.z whisking
// No seperators each value is 5 in length.
void send_radio_event()
{  
  String value = GetPaddedString(MPU.getAngleX( true ));
  value += GetPaddedString(MPU.getAngleY( true ));
  value += GetPaddedString(MPU.getAngleZ( true ));
  value += GetPaddedString( whisking );
  
  char message[21] = "";

  value.toCharArray(message, 21);
  
  rfm.send( (uint8_t*) message, strlen(message) );
  rfm.waitPacketSent();
  if( debug_serial )
  {
    Serial.print("Sent ");
    Serial.println(message);
  }
  
}

void receive_radio_event()
{
  
  if( rfm.available() )
  {

    uint8_t buff[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buff);

    if( rfm.recv(buff, &len) )
    {
      buff[len] = 0;
      if (buff[0] == 'N')
        MPU.normalize();
    }
    
  }
}

void rfm_reset()
{
  digitalWrite(RFM_RES, HIGH);
  delay(10);
  digitalWrite(RFM_RES, LOW);
  delay(10);
}

void UpdateWhisking()
{

  whiskValue = analogRead(WHISK_INPUT);

  int currentValue = whiskValue;
  bool valueIsLow = currentValue < whisking_switch_lowValue;
  
  if(valueIsLow != whisking_lastWasLow)
  {
    whisking = true;
    whisking_nextInterval = millis() + whisking_valueChanged_interval;
  }
  else if(millis() >= whisking_nextInterval)
  {
    whisking = false;
    whisking_nextInterval = millis() + whisking_valueChanged_interval; 
  }

  whisking_lastWasLow = valueIsLow;

  if( debug_serial )
  {
    Serial.print("Whisking: ");
    Serial.println( whiskValue );
  }
  
}

void debuging()
{

  if( Serial.available() > 0 )
  {
    char incoming = Serial.read();
    
    if ( incoming == 'd' || incoming == 'D' )
      debug_serial = incoming == 'D';
  }
  
}
