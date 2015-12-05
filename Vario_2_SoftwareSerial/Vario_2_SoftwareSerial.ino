/*
Links:
-> GoFly
https://sites.google.com/site/jarosrwebsite/para-nav
-> Vario Algorithm
http://taturno.com/2011/10/30/variometro-la-rivincita/   
-> Evitar que o delay atrofie com o loop.
http://www.daqq.eu/index.php?show=prj_sanity_nullifier 
-> Voltmeter
http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
 */

#include <Wire.h>				//i2c library
#include <Adafruit_BMP085.h>	//bmp085 barometric sensor library
#include <toneAC.h>             //tone library
#include <stdlib.h>             
#include <SoftwareSerial.h>


/////////////////////////////////////////
// Variables
///////////////////////////////////////// 
short redPin = 11;						//pin + led battery state_1
short greenPin = 12;
#define NMEA_LED_pin 6 						//pin + led battery state_2
//short speaker_pin1 = 9;   //9 	        //arduino speaker output -
//short speaker_pin2 = 10;    //10         //arduino speaker output +
float vario_climb_rate_start = 0.5;    //minimum climb beeping value(ex. start climbing beeping at 0.4m/s)
float vario_sink_rate_start = -0.5;    //maximum sink beeping value (ex. start sink beep at -1.1m/s)
									   //numa asa com maior planeio isto é capaz de precisar de ser configurado.  -0.95m/s ?
#define SAMPLES_ARR 25                 //define moving average filter array size (2->30), more means vario is less sensitive and slower, NMEA output
#define UART_SPEED 9600                //define serial transmision speed (9600,19200, etc...)
#define PROTOCOL 2                      //define NMEA output: 1 = $LK8EX1, 2 = FlymasterF1, 0 = both
            // LED NMEA out pin 
#define NMEA_OUT_per_SEC 3             // NMEA output string samples per second (1 to 20)
#define VOLUME 2                       // volume 0-no sound, 1-low sound volume, 2-high sound volume (ALTERAR!)

SoftwareSerial mySerial(2, 3); // RX, TX
/////////////////////////////////////////
/////////////////////////////////////////
Adafruit_BMP085 bmp085;
long     Temperature = 0;
long     Pressure = 101325;                 // 1 Atm = 101325 Pa 
float    Altitude=0;
float    Battery_Vcc_mV = 0;            	//Vcc from battery em mV
float    Battery_Vcc_V = 0;             	//Vcc from battery em V
const float p0 = 101325;             		//Pressure at sea level (Pa)
unsigned long timestamp01 = millis();		//guarda timestamp inicial
unsigned long timestamp02 = millis();		//timer for tempmeasure (checkTempEveryMs)
unsigned long timestamp03 = millis();		//timer for NMEA_OUT
unsigned long timestamp04 = millis();		//timer for batterycheck (checkBattEveryMs)
int      my_temperature = 1;
char     altitude_arr[6];					//array alt string 
char     vario_arr[6];						//array vario string 
char     batt_arr[6];						//array battery string 
char     pressure_arr[10];					//array pressure string

int      samples=40;
int      maxsamples=50;
float    alt[51];
float    tim[51];
float    beep;
float    Beep_period;
static long k[SAMPLES_ARR];
int 	checkTempEveryMs=1000;				// read temp every X ms
int     checkBattEveryMs=1000;				// check battery every X ms

boolean bluetooth_alive	= false;					//check for bluetooth available
boolean debugmode = false;
float VarioSim = 1;
int analogInput = 0;

static long Averaging_Filter(long input);
static long Averaging_Filter(long input)	// moving average filter function
{
  long sum = 0;
  for (int i = 0; i < SAMPLES_ARR; i++) {
    k[i] = k[i+1];
  }
  k[SAMPLES_ARR - 1] = input;
  for (int i = 0; i < SAMPLES_ARR; i++) {
    sum += k[i];
  }
  return ( sum / SAMPLES_ARR ) ;
}

void play_welcome_beep()					//play only once welcome beep after turning on arduino vario
{
 for (int aa=300;aa<=1500;aa=aa+100)
  {                
    toneAC(500, 1, 2000, true);			// play beep on pin (note,duration)                
    delay(100);
  }
  for (int aa=1500;aa>=100;aa=aa-100)		// play beep on pin (note,duration)
  {
 	 toneAC(700, 1, 2000, true);			// play beep on pin (note,duration)
     delay(100);
  }
}
// setup() function to setup all necessary parameters before we go to endless loop() function
void setup()
{
  Serial.begin(UART_SPEED);				// set up arduino serial port  
  mySerial.begin(9600);					// set the data rate for the SoftwareSerial port
  Wire.begin();							// lets init i2c protocol
  
	pinMode(redPin, OUTPUT);			// sets the digital pin as output
	pinMode(greenPin, OUTPUT);			// sets the digital pin as output 
	pinMode(NMEA_LED_pin, OUTPUT);		// set pin for output NMEA LED blink;
	digitalWrite(greenPin, HIGH);
	digitalWrite(redPin, HIGH);
	pinMode(analogInput, INPUT); 
	
	play_welcome_beep();				//everything is ready, play "welcome" sound
	

  if (!bmp085.begin()) {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	while (1) {}
  }                  
 
}

//Init Variables
float nmea_vario_cms =0;
float nmea_time_s=0;
float nmea_alt_m=0;
float nmea_old_alt_m=0;

void loop(void)
{
  float time=millis();					//take time, look into arduino millis() function
  float vario=0;
  float N1=0;
  float N2=0;
  float N3=0;
  float D1=0;
  float D2=0;
  
  Pressure = bmp085.readPressure();
  long average_pressure = Averaging_Filter(Pressure);                   //put it in filter and take average, this averaging is for NMEA output
  Altitude = (float)44330 * (1 - pow(((float)Pressure/p0), 0.190295));  //take new altitude in meters from pressure sample, !!NOT FROM!! average pressure
 
 nmea_alt_m = (float)44330 * (1 - pow(((float)average_pressure/p0), 0.190295));	//take new altitude in meters from pressure sample, !!FROM!! average pressure
 if ((millis() >= (nmea_time_s+1000))){
 nmea_vario_cms = ((nmea_alt_m-nmea_old_alt_m))*100; 
 nmea_old_alt_m = nmea_alt_m;
 nmea_time_s = millis();
 }
 
 
 //Check Battery
if (millis() >= (timestamp02+checkTempEveryMs))		//every 10 second get battery level
//if (millis() >= (timestamp04+checkTempEveryMs))
{							
	Battery_Vcc_V = (analogRead(analogInput)*0.0049);	//get voltage in volts	
	timestamp04 = millis();

	//Serial.println(Battery_Vcc_V);
	
	//Display battery state via LED
	if(Battery_Vcc_V > 3.60)
	{
		digitalWrite(greenPin, HIGH);
		digitalWrite(redPin, LOW);
	}
	if(Battery_Vcc_V > 3.40 && Battery_Vcc_V < 3.60)
	{
		digitalWrite(greenPin, HIGH);
		digitalWrite(redPin, HIGH);
	}
	if(Battery_Vcc_V <= 3.40)
	{
		digitalWrite(greenPin, LOW);
		digitalWrite(redPin, HIGH);
	}
}

//check Temp

if (millis() >= (timestamp02+checkTempEveryMs))		
	{
		Temperature = bmp085.readTemperature();
		my_temperature = Temperature;
		timestamp02 = millis();	
	
		//Check BlueTooth available
			if( mySerial.available() )       // if data is available to read
				{
				bluetooth_alive = true;			
				}
			else
				{
				bluetooth_alive = false;
				}
			if (debugmode)						
				{
					if (VarioSim >= 15)
					{
						VarioSim = 0;
					} 
				VarioSim++;								//for(float aa=1;aa<=15;aa++){	
				} 
			
	}
	
if (digitalRead(NMEA_LED_pin)== HIGH) {digitalWrite(NMEA_LED_pin, LOW);} else {digitalWrite(NMEA_LED_pin, HIGH);}

 // Vario algorithm
  for(int cc=1;cc<=maxsamples;cc++){
    alt[(cc-1)]=alt[cc];						//altidude to array
    tim[(cc-1)]=tim[cc];						//time to array
  }; 
  alt[maxsamples]=Altitude;						//altidude to array
  tim[maxsamples]=time;							//time to array
  float stime=tim[maxsamples-samples];
  for(int cc=(maxsamples-samples);cc<maxsamples;cc++){
    N1+=(tim[cc]-stime)*alt[cc];
    N2+=(tim[cc]-stime);
    N3+=(alt[cc]);
    D1+=(tim[cc]-stime)*(tim[cc]-stime);
    D2+=(tim[cc]-stime);
  };
//SYNTAX:
//   toneAC( frequency [, volume [, length [, background ]]] ) - Play a note.
//     Parameters:
//       * frequency  - Play the specified frequency indefinitely, turn off with toneAC().
//       * volume     - [optional] Set a volume level. (default: 10, range: 0 to 10 [0 = off])
//       * length     - [optional] Set the length to play in milliseconds. (default: 0 [forever], range: 0 to 2^32-1)
//       * background - [optional] Play note in background or pause till finished? (default: false, values: true/false)
//   toneAC()    - Stop playing.
//   noToneAC()  - Same as toneAC().
//Test

							//time to array
if (debugmode)
	{
	vario = VarioSim;
	}
else
	{
	vario=1000*((samples*N1)-N2*N3)/(samples*D1-D2*D2);		
	}	
 if ((time-beep)>Beep_period)                          
	 {	 
		beep=time;
		if (vario>vario_climb_rate_start && vario<=10 && !bluetooth_alive && (VOLUME > 0) )
		{
			if (VOLUME == 1 )
			{
				Beep_period=550-(vario * 5);			 
				//toneAC((1000 + (100 * vario)), 2, 50, true);
				//toneAC((1000 + (100 * vario)), 2, 300 - (vario * 5), true);
				toneAC((1400+(200*vario)), 2, 420-(vario*(20+vario)), true);
			}
			else
			{
				Beep_period=550-(vario*(30+vario));			 
				toneAC((1400+(200*vario)), 10, 420-(vario*(20+vario)), true);			 
			 }
		}
		else if (vario >10 && !bluetooth_alive && (VOLUME > 0))
		{
			if (VOLUME == 1 )
			{
				Beep_period=160;			 
				toneAC(3450, 5, 120, true);
			}
			else
			{
				Beep_period=160;			 
				toneAC(3450, 10, 120, true);		
			}
		}
		else if (vario< vario_sink_rate_start && !bluetooth_alive)
		{
			if (VOLUME ==1 )
			{
				Beep_period=600;			 
				toneAC(300, 5, 340, true);
			}
			else
			{
				Beep_period=600;			 
				toneAC(300, 10, 340, true);			 
			}
		}
	} 


  

//LK8000
//  $LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
//	pressure in hPascal:hPA*100 (example for 1013.25 becomes 101325) no padding (987.25 becomes 98725, NOT 098725)
// If no pressure available, send 999999 (6 times 9)
// If pressure is available, field 1 altitude will be ignored
// Field 1, altitude in meters, relative to QNH 1013.25
//  If raw pressure is available, this value will be IGNORED (you can set it to 99999
// but not really needed)!(if you want to use this value, set raw pressure to 999999)
//  This value is relative to sea level (QNE). We are assuming that currently at 0m
// altitude pressure is standard 1013.25.If you cannot send raw altitude, then send
//  what you have but then you must NOT adjust it from Basic Setting in LK.
//  Altitude can be negative. If altitude not available, and Pressure not available, set Altitude
//  to 99999. LK will say "Baro altitude available" if one of fields 0 and 1 is available.
//  Field 2, vario in cm/s
//  If vario not available, send 9999. Value can also be negative.
//  Field 3, temperature in C , can be also negative.If not available, send 99
//  Field 4, battery voltage or charge percentage.Cannot be negative.If not available, send 999.
// Voltage is sent as float value like: 0.1 1.4 2.3 11.2. To send percentage, add 1000.
// Example 0% = 1000. 14% = 1014 . Do not send float values for percentages.
// Percentage should be 0 to 100, with no decimals, added by 1000!
 
if ((millis() >= (timestamp03+(1000/NMEA_OUT_per_SEC))) && (PROTOCOL == 1))       
  {
    int battery_percentage = map(Battery_Vcc_mV, 2500, 4300, 1, 100);  
    String str_out =                                                                 
      String("LK8EX1"+String(",")+String(average_pressure,DEC)+String(",")+String(dtostrf(Altitude,0,0,altitude_arr))+String(",")+
      String(dtostrf(nmea_vario_cms,0,0,vario_arr))+String(",")+String(my_temperature,DEC)+String(",")+String(battery_percentage+1000)+String(","));
    unsigned int checksum_end,ai,bi;
    for (checksum_end = 0, ai = 0; ai < str_out.length(); ai++)
    {
      bi = (unsigned char)str_out[ai];
      checksum_end ^= bi;
    }
    Serial.print("$");                     //print first sign of NMEA protocol
    Serial.print(str_out);                 // print data string
    Serial.print("*");                     //end of protocol string
    Serial.println(checksum_end,HEX);      //print calculated checksum on the end of the string in HEX
    timestamp03 = millis();
    if (digitalRead(NMEA_LED_pin)== HIGH) {digitalWrite(NMEA_LED_pin, LOW);} else {digitalWrite(NMEA_LED_pin, HIGH);}
   }

  //Flymaster F1 
  //$VARIO,fPressure,fVario,Bat1Volts,Bat2Volts,BatBan k,TempSensor1,TempSensor2*CS
  
  //Where:
  //fPressure = the absolute atmospheric pressure, converting to altitude use the following:
  //fBarAltitude = (1 - pow(fabs(fPressure / fQNH), 0.190284)) * 44307,69;        
  //fVario = the variometer in decimeters per second
  //Bat1Volts = the voltage of the battery in bank 1
  //Bat2Volts = the voltage of the battery in bank 2
  //BatBank = the battery bank in use.
  //TempSensor1 = temperature in ?C of external wireless sensor 1
  //TempSensor2 = temperature in ?C of external wireless sensor 2
  //CS = the standard NMEA checksum.

  if ((millis() >= (timestamp03+(1000/NMEA_OUT_per_SEC))) && (PROTOCOL == 2)) 
  {   
    //signed
    /*
    String str_out =
    String("VARIO,"+                                      //$VARIO
    String(dtostrf(((float)average_pressure/100),2,2,pressure_arr))+ //fPressure
    String(",")+
    String(dtostrf((nmea_vario_cms/10),2,2,vario_arr))+   //fVario
    String(",")+                                   
    String(dtostrf((Battery_Vcc_V),2,2,batt_arr))+  //Bat1Volts
    String(",0,1")+                                   //Bat2Volts,BatBan k,
    String(",")+
    String(my_temperature,DEC)+                           //TempSensor1
    String(",0"));                                        //TempSensor2
    */
    //unsigned
    String str_out =
    String("VARIO,"+													//$VARIO
    String(dtostrf(((float)average_pressure/100),0,0,pressure_arr))+	//fPressure
    String(",")+
    String(dtostrf((nmea_vario_cms/10),2,2,vario_arr))+					//fVario
    String(",")+                                   
    String(dtostrf((Battery_Vcc_V),2,2,batt_arr))+						//Bat1Volts
    String(",0,1")+														//Bat2Volts,BatBan k,
    String(",")+
    String(my_temperature,DEC)+											//TempSensor1
    String(",0"));														//TempSensor2
//VARIO,999.98,-12,12.4,12.7,0,21.3,25.5*66
		
    unsigned int checksum_end,ai,bi;
    for (checksum_end = 0, ai = 0; ai < str_out.length(); ai++)
    {
      bi = (unsigned char)str_out[ai];
      checksum_end ^= bi;
    }
	//Print Serial
    Serial.print("$");
    Serial.print(str_out);
    Serial.print("*");
    Serial.println(checksum_end,HEX);									//checksum
    //Print BlueTooth
    mySerial.print("$");
    mySerial.print(str_out);
    mySerial.print("*");
    mySerial.println(checksum_end,HEX);									//checksum
    
    timestamp03 = millis();
    if (digitalRead(NMEA_LED_pin)== HIGH) 
    {
      digitalWrite(NMEA_LED_pin, LOW);
    }
    else
    {
      digitalWrite(NMEA_LED_pin, HIGH);
    }
  }
}

