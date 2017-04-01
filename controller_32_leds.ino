/************************************
*      Controller for 32 LEDs       *
*      Arduino ProMicro Program     *
*         12/18/2016                *
************************************/
/*
Name: Controller for 32 LEDs
Synopsis: This code implements a controller for 32 LEDs.  The
LEDs can be used in various types of displays.
Description:  See ?? for a discription of the project.
Requires: Compile program with Arduino IDE, www.arduino.cc.

Depending on the hardware used in the project, edit the following to 
to conditional compile the code.
#define NINE_DOF set to 1 or 0 for TRUE or FALSE
#define NEOPIXEL set to 1 or 0 for TRUE or FALSE
#define PROMICRO set to 1 or 0 for TRUE or FALSE,  FALSE for UNO
#define RTC set to 1 or 0 for TRUE or FALSE
#define TEMT6000 set to 1 or 0 for TRUE or FALSE

Hardware:  The code in this module controls the following hardware.
But not all tis hardware is required for the project.
Pro-Micro MCU 5V/16MHz
RTC module
TEMT6000 breakout module
9DOF stick w/ PCA9306 voltage level converter
TLC5940 Breakout
NEOPIXEL stick
Hardware configuration:
Pro-Mico: clear SJ1 to prevent Vcc and UVcc from being shorted together.
RTC module: Clear SJ2 to disable the I2C pull ups since the pull ups are on the PCA9306 module. 
Neopixel stick

Pro-Mico configuration
Pin: function: connection
1: TXO/D1: RXI from Bluefruit BLE
2: RXI/D0: TXO from Bluefruit BLE
3: GND: ground
4: GND: ground
5: D2: I2C SDA
6: D3: I2C SCL
7: D4/A6: TLC5940 XLAT
8: D5: SQW
9: D6/A7: SW1
10: D7: DIN on neopixel stick
11: D8/A8: TLC5940 VPRG
12: D9/A9: TLC5940 GSCLK

13: D10/A10: TLC5940 Blank
14: D16/MOSI: TLC5940 SIN
15: D14/MISO: 
16: D15/SCK: TLC5940 SCLK
17: A0/TDI: TEMT6000
18: A1/TDO: NC
19: A3/TMS: NC
20: A3/TCK: NC
21: Vcc +5VDC output: Vcc to other modules
22: Reset: NC
23: GND: ground
24: Raw 6 to 9 volts input: power input from external power supply 

Links to example code
RTC
https://learn.sparkfun.com/tutorials/real-time-clock-module-hookup-guide?_ga=1.138338672.1508991003.1478267692

TLC5940
https://www.sparkfun.com/products/10616?_ga=1.221751512.1508991003.1478267692
Controller for 32 LEDs using a TLC5940 breakout module from sparkfun.
The libraries from Sparkfun didn't run out of the box and digging into the code
did provide any easy fixes given my limited experience with using classes.
I'll be writing new code using bit banging to control the data and clock lines.
notes:
Put the longer leg (anode) of the LEDs in the +5V and the shorter leg (cathode) in OUT(0-15).

Neopixel
https://www.adafruit.com/products/1426
https://github.com/adafruit/Adafruit_NeoPixel

9DOF
https://www.sparkfun.com/products/13944
https://learn.sparkfun.com/tutorials/9dof-sensor-stick-hookup-guide?_ga=1.217964758.1508991003.1478267692

This is a work in progress and more detail will be added later.
The schematic can be found here:
https://easyeda.com/tcirineo/Controller_for_32_LEDs-w9JtnhD9h

Author: Tony Cirineo
Date: 12/18/2016 coding started
Revision History
Original version:  

*/

/* Conditional Compile, set to 1 or 0 depending on the harware */
#define NINE_DOF 0
#define NEOPIXEL 0
#define PROMICRO 1
#define RTC 0
#define TEMT6000 1

/* Includes */
#if RTC
#include "Wire.h"
#endif
#if NEOPIXEL
#include <Adafruit_NeoPixel.h>
#endif
#if NINE_DOF
#include <SparkFunLSM9DS1.h>
#endif

typedef unsigned char byte;

/* defines */
#if RTC
#define DS1307_ADDRESS 0x68
byte zero = 0x00; //workaround for issue #527
#endif

#if NEOPIXEL
#define PIN 7  // Which pin on the Arduino is connected to the NeoPixels?
#define NUMPIXELS 8  // How many NeoPixels are attached to the Arduino?
#endif

#if NINE_DOF
// SDO_XM and SDO_G are both pulled high, default addresses are:
#define LSM9DS1_M	0x1E
#define LSM9DS1_AG	0x6B

#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.
#endif

// create instances
#if NEOPIXEL
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#endif
#if NINE_DOF
LSM9DS1 imu;
#endif

/************************************
*         PORT SIGNALS              *
************************************/
// port names are the same as the net names in the schematic
// The RX LED has a defined Arduino pin, used for testing because it's connected to a yellow LED
#define RXLED 17  
// switch 1, for selecting modes
#define SW1 6  
// neopixel stick data port
#define DIN 7
// square wave output from DS1307, real time clock
#define SQW 5  

// test point for measuring timing between events
#define TP1 14 

// TLC5940 control signals
#define BLANK 10
#define GSCLK 9
#define XLAT 4
#define VPRG 8
#define SIN 16
#define SCLK 15

// analog I/O pins
//Ambient light sensor reading TEMT6000
#define temt6000Pin A0 

/************************************
*         GLOBAL VARIABLES          *
************************************/
#if RTC
int second;
int minute;
int hour; 		// 24 hour time
int weekDay; 	// 1-7 = sunday - Saturday
int monthDay;
int month;
int year;
#endif

int light_lvl;
unsigned char led_on_flg;

#define NUM_TLCS 2
/*
full 16 or 8 bits are used to store the data, only 12 or 6 are extracted when shifting out
Grey scale: 16 12bit words = 192 bits in serial data message
Dot correction: 16 6bit words = 96 bits in serial data message
*/
unsigned int tlc_GSData[NUM_TLCS * 16]; 
byte tlc_DCData[NUM_TLCS * 16];

byte r_flg, s_flg;  // used to keep track of the phase of the SQW 1Hz signal

// this structure holds the info for the light list
struct {
    byte intensity_correction;      //used to normalize the intensity of the LEDs
    byte seq_list[10];      //a list of on/off durations starting with 1st on duration
    byte t_start_hr;        // can be used to limit operation of LEDs
    byte t_start_min;
    byte t_stop_hr;
    byte t_stop_min;
    byte night_only;
} light[NUM_TLCS * 16];
    
struct {
    unsigned long time;       //number of 0.1 seconds to next event 
    byte seq_list_ptr; // points to a position in the duration list
} event_list[NUM_TLCS * 16];

#if NEOPIXEL
// neopixel
struct {
    byte red;
    byte blue;
    byte green;
} np_array[NUMPIXELS];
byte temp_array[NUMPIXELS*3];
#endif

unsigned long currentMillis;
unsigned long lastUpdate, updateInterval;
unsigned long tenths, last_tenths, tenths_interval;

//zig zag
int direction = 1;
int channel = 0;

// triangle_intensity
unsigned int dc_value;
 
/************************************
*              SetUp                *
************************************/
/*
Name: SetUp
Synopsis: this function has the initialization code and 
code to be run only once at power up.
Requires: na
Description: This is function initializes hardware and variables
Author: Tony Cirineo
Date:  12/6/2016
Revision History:
*/
void setup() {
byte i;

    pinMode(RXLED, OUTPUT);  // Set RX LED as an output
    pinMode(SW1, INPUT);
    pinMode(SQW, INPUT);
    pinMode(TP1, OUTPUT);
	pinMode(temt6000Pin, INPUT);

#if RTC    
    Wire.begin();
    //Serial.begin(9600); //300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, or 115200.
    Serial.begin(115200);
#endif

#if NEOPIXEL
    // initialize neopixel strip to off
    pixels.begin(); // This initializes the NeoPixel library.
    turnoff_neopixel_stick();
#endif

    // init TLC5940
	pinMode(BLANK, OUTPUT);
	pinMode(GSCLK, OUTPUT);     
	pinMode(XLAT, OUTPUT);    
	pinMode(VPRG, OUTPUT);    
	pinMode(SIN, OUTPUT);    
	pinMode(SCLK, OUTPUT);    
    Tlc5940_init();

#if NINE_DOF    
    // init LSM9DS1
    imu.settings.device.commInterface = IMU_MODE_I2C;
    imu.settings.device.mAddress = LSM9DS1_M;
    imu.settings.device.agAddress = LSM9DS1_AG;
    imu.begin();
#endif

    randomSeed(analogRead(temt6000Pin));  //used to populate the LED start times
    led_on_flg = 1;
    r_flg = 0;  // used to keep track of edges on the 1 Hz SQW
    s_flg = 0;

    build_light_list();     // populates the light list data structure with data
    build_event_list();     // builds a list of up comming actions for the LEDs
    
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function below
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    updateInterval = 50;  // within the ISR, increment tenths count every 0.1 sec
    tenths = 0;
    last_tenths = 0;
    tenths_interval = 1;    // every 0.1 seconds 
}

/************************************
*             SIGNAL                *
************************************/
/*
Name: signal
Synopsis: This is an interrupt service routine for the
timer.  Updates the tenths of second counter from timer 0
timer0 runs at Timer0 is an 8-bit that counts from 0 to 255 and generates
an interrupt whenever it overflows. It uses a clock divisor of 
64 by default to give us an interrupt rate of 976.5625 Hz.

need to account for current millis roll over
  
Requires: na
Description: 
Author: Tony Cirineo
Date:  
Revision History:
*/
// Interrupt is called once a millisecond
SIGNAL(TIMER0_COMPA_vect) 
{
    currentMillis = millis();
    if((currentMillis - lastUpdate) > updateInterval){  // time to update
      lastUpdate = currentMillis;
      tenths++;
    }
} 

/************************************
*              Loop                 *
************************************/
/*
Name: Loop
Synopsis: This is the main program loop.
Requires: na
Description: Inside this loop, either system test is called
or process event list.
System test just runs various little test programs to light the LEDs.
process event list controls the LED to blink as defined in the light list.
Author: Tony Cirineo
Date:  
Revision History:
*/
void loop() {
int i;

    //check switch and run system and LED tests
    if(digitalRead(SW1)){
        system_test();      //runs system tests and returns when switch is not true
        return;
    }

#if RTC
    // only need to look at the time once per second
    if(digitalRead(SQW) && !s_flg){
        r_flg = 1;
        digitalWrite(RXLED, LOW);
    }

    if(!digitalRead(SQW) && !r_flg){
        s_flg = 0;
        digitalWrite(RXLED, HIGH);
    }
    // read time and date once per second
    if(r_flg){
        getDate();
        if((hour == 4) && (minute == 0) && !led_on_flg){
            led_on_flg = 1;
        } 

        if((hour == 23) && (minute == 50) && led_on_flg){
            led_on_flg = 0;
            for(i=0;i < NUM_TLCS*16;i++){
                tlc_DCData[i] = 0;
            }        
            Tlc5940_set_dc_values();
        } 

        s_flg = 1;
        r_flg = 0;
    }

    if(led_on_flg)
#endif
        process_event_list();
}

/************************************
*       process event list          *
************************************/
/*
Name:
Synopsis:
controls LEDs to blink like nautical navigation lights
A Light List is a detailed list of navigational aids including lighthouses and other 
lighted navigation aids, unlighted lights, radiobeacons, daybeacons, and racons. Light 
lists are published by most of the major maritime nations. Some nations, including the 
United Kingdom and the United States, publish lists that cover the whole world in many 
volumes. Other nations publish lists that cover only their own coasts.

Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void process_event_list(void)
{
int i,ptr;

    //digitalWrite(TP1, HIGH);        //used to signal length of time in this function
    // check the tenths interval then check event list and update LEDs
    if((tenths - last_tenths) > tenths_interval){
        digitalWrite(TP1, HIGH);        //used to signal length of time in this function
        // for each LED decide what action is to be taken
        //i = 1;
        for(i = 0;i < NUM_TLCS*16;i++){

            if (tenths >= event_list[i].time){  // is it time to update the list? 

                // get next action time, check for zero value which incidates end of list
                ptr = event_list[i].seq_list_ptr;
                if(light[i].seq_list[ptr] != 0){
                    event_list[i].time = (unsigned long) light[i].seq_list[ptr] + tenths;    //calculate time to next event and save
                    if(ptr%2 == 0)      //all even entries are LED on events, odd entries are LED off events
                        tlc_DCData[i] = 10;     // add code to normalize LED brightness and ambient control
                    else
                        tlc_DCData[i] = 0;
                }    
                else{   //end of the time list 
                    event_list[i].seq_list_ptr = 0;       //reset the pointer to start of the list
                    event_list[i].time = (unsigned long) light[i].seq_list[0] + tenths;    //get 1st time from the list, calculate time to next event
                    tlc_DCData[i] = 10;
                }
                event_list[i].seq_list_ptr++;       //bump the pointer                    
            }
        }
        last_tenths = tenths; //update 
        // update the dot correction registers
        Tlc5940_set_dc_values();
    }
    digitalWrite(TP1, LOW);        //used to signal length of time in this function    
}

/************************************
*          build event list         *
************************************/
/*
Name: build event list
Synopsis: Creats a todo list of future times to
either turn on or off the LEDs.  
Initalizes the event list from the light list and the start
offset time is a random number between 0 and 5 seconds.
Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void build_event_list(void)
{
int i;

    for(i = 0;i < NUM_TLCS*16;i++){
        event_list[i].seq_list_ptr = 0;
        event_list[i].time = (unsigned long) random(0,50) + tenths;
    }   
}

/************************************
*           system test                *
************************************/
/*
Name:
Synopsis:

Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void system_test(void)
{
//    if(!digitalRead(SW1))
//        return;

    //ambient_control();
    //triangle_intensity();
    //twinkle();
    //random_blink();
    zig_zag();
    //sin_sclk_test();
/*
All LEDs on with intensity stepping up and down
Alternate on/off with sqw
zig zag
display time in morse code w/ ambient control

*/        

}

/************************************
*        sin_sclk_test              *
************************************/
/*
Name: sin sclk test
Synopsis:  controls sin and sclk signals for testing
Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
12/27/2016
Using this function to debug the SIN to SCLK transition.
My scope shows the rising edge of SIN and SCLK to be the same
no matter what delays I use.  Maybe a problem with how my scope 
triggers.
12/29/2016
gave up and working on other things.
*/
void sin_sclk_test(void)
{
unsigned int i, j, n;
byte dc_value, chan_num;

    digitalWrite(BLANK, HIGH);    // turns off all LEDs during this test
    // set all other lines low
#if 1
    digitalWrite(XLAT, LOW);
    digitalWrite(VPRG, LOW);
    digitalWrite(SIN, LOW);
    digitalWrite(SCLK, LOW);   
    digitalWrite(GSCLK, LOW);     
#endif

    // initalized data arrays to some test data
    for(i=0;i < NUM_TLCS * 16;i++){
        tlc_GSData[i] = 0xa5; 
        tlc_DCData[i] = 0xa5;
    }
#if 1
while(1){
    // control the sin and sclk lines
//    chan_num = 0;
//    for(n = 0;n < NUM_TLCS; n++){
//        for(i = 0; i < 16; i++){	// Send 16 words to dot correct registers
            dc_value = 0xa5; //tlc_DCData[chan_num];    //read from array
            for(j = 0; j < 6; j++){	// Send 6 bits for each word
                digitalWrite(SIN, ((dc_value & 0x20) ? HIGH : LOW)); //msb goes 1st
                dc_value  <<= 1; // Shift the byte by one bit
                delay_1();  
                digitalWrite(SCLK, HIGH);   // raise the SCLK line
                //delay(1);              
                digitalWrite(SCLK, LOW);   // lower the SCLK line
                //delay_1(); 
            }
            //chan_num++;
        //}
    //}
    //delay(50);
}
    //digitalWrite(SIN, LOW); //leave data line low
#endif
}

/************************************
*         ambient control           *
************************************/
/*
Name:
Synopsis:  ramps intensity up and down
back and forth pattern.  Two levels of
light control are used.

Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void ambient_control(void)
{
int i;

	//Read and display light level
	light_lvl = analogRead(temt6000Pin);
    if(light_lvl > 50)
        dc_value = 30;
    else
        dc_value = 5;    
    
    for(i=0;i < NUM_TLCS * 16;i++)
        tlc_DCData[i] = dc_value;

    // update the dot correction registers
    Tlc5940_set_dc_values();

    delay(500); 

}

/************************************
*         triangle intensity        *
************************************/
/*
Name:
Synopsis:  ramps intensity up then resets
back to zero.
counts up 0 to 15, then jumps to 63.
Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void triangle_intensity(void)
{
int i;

    for(i=0;i < NUM_TLCS * 16;i++)
        tlc_DCData[i] = dc_value;

    // update the dot correction registers
    Tlc5940_set_dc_values();

    dc_value++;
     if(dc_value >= 63)
        dc_value = 0;
    if(dc_value >= 15)
        dc_value = 63; 
    delay(500);    
}

/************************************
*           zig zag                 *
************************************/
/*
Name: zig zag
Synopsis:
back and forth pattern

Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void zig_zag(void)
{
int i;

    for(i=0;i < NUM_TLCS * 16;i++)
        tlc_DCData[i] = 0;

    if (channel == 0)
        direction = 1;
    else
        tlc_DCData[channel-1] = 31;

    tlc_DCData[channel] = 63;    
    
    if (channel != NUM_TLCS * 16 - 1)
        tlc_DCData[channel-1] = 31;        
    else
      direction = -1;

    // update the dot correction registers
    Tlc5940_set_dc_values();
    
    delay(100);
    channel += direction;    
}

/************************************
*           fire fly                *
************************************/
/*
Name: fire fly
Synopsis: suppost to emulate a fire fly
fire fly pattern
on starting led, increase the intensity and move 
4 positions and decress intensity

Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void fire_fly(void)
{
}

/************************************
*           twinkle                 *
************************************/
/*
Name: twinkle
all LEDs on
intensity varies in a random fasion for a random time
Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void twinkle(void)
{
int randNumber, i, j;

    // control the TLC5940 LEDs

    //get a random number between 0 and 31
    randNumber = random(16); // turn on this many LEDs   
    for(i = 0;i < randNumber;i++){
        // set a random position to a random value brightness
        tlc_DCData[random(32)] = random(1,10);    
    }
    Tlc5940_set_dc_values();

#if NEOPIXEL
    //-----------------------------------------------
    // add neopixel stick to twinkle, 3x8=24 LEDs

    //get a random number
    randNumber = random(16); // turn on this many LEDs   
    for(i = 0;i < randNumber;i++){
        // set a random position to a random value brightness
        temp_array[random(23)] = random(0,5);    
    }    
    for(i = 0,j = 0;i < NUMPIXELS;i++){    
        np_array[i].red = temp_array[j];
        np_array[i].blue = temp_array[j++];
        np_array[i].green = temp_array[j++];
    }

    for(int i=0;i<NUMPIXELS;i++){
        // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
        pixels.setPixelColor(i, pixels.Color(np_array[i].red,np_array[i].green,np_array[i].blue));
        pixels.show(); // This sends the updated pixel color to the hardware.
    }
#endif
    delay(random(10,300));
}

/************************************
*           Random blink            *
************************************/
/*
Name: random blink
turns on random LEDs for a random duration and intensity
Synopsis:
Using the random number generator, seeded with the ambient light
reading, do the following:
1) turn on random LEDs for a random time
2) intensity is also random
Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void random_blink(void)
{
int randNumber, i, j;

    // control the TLC5940 LEDs
    //get a random number between 0 and 31
    randNumber = random(16); // turn off this many LEDs   
    for(i = 0;i < randNumber;i++){
        // set a random position to off
        tlc_DCData[random(32)] = 0;    
    }

    //get a random number between 0 and 31
    randNumber = random(3); // turn on this many LEDs   
    for(i = 0;i < randNumber;i++){
        // set a random position to a random value brightness
        tlc_DCData[random(32)] = random(3,10);    
    }
    Tlc5940_set_dc_values();

#if NEOPIXEL
    //-----------------------------------------------
    // add neopixel stick to twinkle, 3x8=24 LEDs
    //get a random number
    randNumber = random(12); // turn off this many LEDs   
    for(i = 0;i < randNumber;i++){
        // set a random position to off
        temp_array[random(24)] = 0;    
    }
    //get a random number
    randNumber = random(6); // turn on this many LEDs   
    for(i = 0;i < randNumber;i++){
        // set a random position to a random value brightness
        temp_array[random(23)] = random(1,15);    
    }    
    for(i = 0,j = 0;i < NUMPIXELS;i++){    
        np_array[i].red = temp_array[j];
        np_array[i].blue = temp_array[j++];
        np_array[i].green = temp_array[j++];
    }

    for(int i=0;i<NUMPIXELS;i++){
        // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
        pixels.setPixelColor(i, pixels.Color(np_array[i].red,np_array[i].green,0));
        pixels.show(); // This sends the updated pixel color to the hardware.
    }
#endif
    delay(random(10,300));
}

/************************************
*          Tlc5940_init             *
************************************/
/*
Name: Tlc5940_init
Synopsis: initalize the TLC5940

Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void Tlc5940_init(void)
{
byte i;

    digitalWrite(BLANK, LOW);    // turns on all LEDs during initization
    // set all other lines low
    digitalWrite(XLAT, LOW);
    digitalWrite(VPRG, LOW);
    digitalWrite(SIN, LOW);
    digitalWrite(SCLK, LOW);   
    digitalWrite(GSCLK, LOW); 

    //raise & lower BLANK to reset GS count 
    digitalWrite(BLANK, HIGH);
    delay_1(); 
    digitalWrite(BLANK, LOW);    
    
    // reset bit counters in the TLC5940
    digitalWrite(VPRG, HIGH); 
    delay_1();    
    //raise & lower XLAT to reset bit count 
    digitalWrite(XLAT, HIGH);
    delay_1(); 
    digitalWrite(XLAT, LOW);
    delay_1(); 
    digitalWrite(VPRG, LOW);
    delay_1(); 
    //raise & lower XLAT to reset bit count 
    digitalWrite(XLAT, HIGH);
    delay_1(); 
    digitalWrite(XLAT, LOW);

    // initalized data arrays to some default values
    for(i=0;i < NUM_TLCS * 16;i++){
        tlc_GSData[i] = 10; 
        tlc_DCData[i] = 0;
    }
    
    // initialize the dot correction registers
    Tlc5940_set_dc_values();

    // initialize the grey scale registers, for now set to 0x00f
    Tlc5940_set_gs_values();
    digitalWrite(BLANK, LOW);    // turns on all LEDs
    delay_1();

    // cycle GS clock once
    digitalWrite(GSCLK, HIGH); 
    delay_1();    
    digitalWrite(GSCLK, LOW);     
}

/************************************
*      Tlc5940_set_dc_values        *
************************************/
/*
Name: Tlc5940_set_dc_values
Synopsis: writes values to the dot correction registers

Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void Tlc5940_set_dc_values(void)
{
int i, j, n;
byte dc_value, chan_num;

    // set all other lines low
    digitalWrite(XLAT, LOW);
    digitalWrite(VPRG, HIGH);
    delay_1(); 
    digitalWrite(SIN, LOW);
    digitalWrite(SCLK, LOW);   

    chan_num = 0;
    for(n = 0;n < NUM_TLCS; n++){
        for(i = 0; i < 16; i++){	// Send 16 words to dot correct registers
            dc_value = tlc_DCData[chan_num];    //read from array    
            for(j = 0; j < 6; j++){	// Send 6 bits for each word
                digitalWrite(SIN, ((dc_value & 0x20) ? 1 : 0)); //msb goes 1st
                dc_value  <<= 1; // Shift the byte by one bit
                delay_1();  
                digitalWrite(SCLK, HIGH);   // raise the SCLK line
                delay_1();             
                digitalWrite(SCLK, LOW);   // lower the SCLK line
                //delay_1(); 
            }
            chan_num++;
        }
    }
                
    // latch data to the DOT registers
    digitalWrite(XLAT, HIGH);
    delay_1(); 
    digitalWrite(XLAT, LOW);   
    digitalWrite(VPRG, LOW);
    delay_1();     
}

/************************************
*      Tlc5940_set_gs_values        *
************************************/
/*
Name: Tlc5940_set_gs_values
Synopsis: writes values to the grey scale registers
Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void Tlc5940_set_gs_values(void)
{
unsigned int i, j, n,gs_value;
byte chan_num;

    // set all other lines low
    digitalWrite(XLAT, LOW);
    digitalWrite(VPRG, LOW);
    digitalWrite(SIN, LOW);
    digitalWrite(SCLK, LOW);   
    delay_1(); 

    chan_num = 0;
    for(n = 0;n < NUM_TLCS; n++){
        for(i = 0; i < 16; i++){	// Send 16 words to dot correct registers
            gs_value = tlc_GSData[chan_num];    //read from array    
            for(j = 0; j < 12; j++){	// Send 12 bits for each word
                digitalWrite(SIN, ((gs_value & 0x0800) ? 1 : 0)); //msb goes 1st
                gs_value  <<= 1; // Shift the byte by one bit
                delay_1();  
                digitalWrite(SCLK, HIGH);   // raise the SCLK line
                delay_1();             
                digitalWrite(SCLK, LOW);   // lower the SCLK line
                //delay_1();                  
            }
            chan_num++;
        }
    }

    // latch data to the DOT registers
    digitalWrite(XLAT, HIGH);
    delay_1(); 
    digitalWrite(XLAT, LOW);   
    digitalWrite(VPRG, LOW); 
    delay_1(); 
}

/************************************
*        delay_1                    *
************************************/
/*
Name: 
Synopsis: uses a for loop to creat a small delay
Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void delay_1(void)
{
unsigned int i,t;
	for(i = 0;i < 100;i++);	// loop for a small delay
        //t=t+11;
    //delay(1);
}

#if NINE_DOF
/************************************
*             printGyro             *
************************************/
/*
Name: printGyro
Synopsis: 
Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void printGyro()
{
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  imu.readGyro();
  
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

/************************************
*            printAccel             *
************************************/
/*
Name: printAccel
Synopsis: 
Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  imu.readAccel();
  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW 
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}

/************************************
*              printMag             *
************************************/
/*
Name: printMag
Synopsis: 
Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void printMag()
{
  // To read from the magnetometer, you must first call the
  // readMag() function. When this exits, it'll update the
  // mx, my, and mz variables with the most current data.
  imu.readMag();
  
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

/************************************
*              printAttitude        *
************************************/
/*
Name: printAttitude
Synopsis: 
Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}
#endif

#if NEOPIXEL
/************************************
*            neopixel_test          *
************************************/
/*
Name: neopixel_test
Synopsis: 
Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void neopixel_test(void)
{
#define lvl 1
#define delayval 1

  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
  for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(lvl,0,0)); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(delayval); // Delay for a period of time (in milliseconds).

  }
  
      //delay(2000);              // wait for X second
   for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,lvl,0)); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(delayval); // Delay for a period of time (in milliseconds).

  } 
  
        //delay(2000);              // wait for X second
   for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,0,lvl)); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(delayval); // Delay for a period of time (in milliseconds).

  } 

          //delay(2000);              // wait for X second
   for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(lvl,lvl,lvl)); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(delayval); // Delay for a period of time (in milliseconds).

  } 

          //delay(2000);              // wait for X second
   for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(delayval); // Delay for a period of time (in milliseconds).

  }
}

/************************************
*     turn off neopixel stick       *
************************************/
/*
Name: turnoff neopixel stick
Synopsis: 
Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void turnoff_neopixel_stick(void)
{
int i;
   for(int i=0;i<NUMPIXELS;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.
  }
}
#endif

#if RTC
/************************************
*      Dec & BCD Conversions        *
************************************/
/*
Name: 
Synopsis: 
Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
byte decToBcd(byte val){
// Convert normal decimal numbers to binary coded decimal
  return ( (val/10*16) + (val%10) );
}

byte bcdToDec(byte val)  {
// Convert binary coded decimal to normal decimal numbers
  return ( (val/16*10) + (val%16) );
}

/************************************
*           getDate                 *
************************************/
/*
Name: getDate
Synopsis:
Requires: na
Description:
Author: Tony Cirineo
Date:
Revision History:
*/
void getDate(){

  // Reset the register pointer
  Wire.beginTransmission(DS1307_ADDRESS);

  byte zero = 0x00;
  Wire.write(zero);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDRESS, 7);

  second = bcdToDec(Wire.read());
  minute = bcdToDec(Wire.read());
  hour = bcdToDec(Wire.read() & 0b111111); //24 hour time
  weekDay = bcdToDec(Wire.read()); //0-6 -> sunday - Saturday
  monthDay = bcdToDec(Wire.read());
  month = bcdToDec(Wire.read());
  year = bcdToDec(Wire.read());
}
#endif

/************************************
*         build light list          *
************************************/
/*
Name:
Synopsis:
puts light list data into structure

Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/
void build_light_list(void)
{
    // TLC5940 #1 --------------------------------
    // #0
    light[0].intensity_correction = 0;

    light[0].seq_list[0] = 10;    // A dot
    light[0].seq_list[1] = 10;
    light[0].seq_list[2] = 30;    // dash
    light[0].seq_list[3] = 30;    // char space
    light[0].seq_list[4] = 0;
    light[0].seq_list[5] = 0;
    light[0].seq_list[6] = 0;    
    light[0].seq_list[7] = 0;
    light[0].seq_list[8] = 0;
    light[0].seq_list[9] = 0;
    
    light[0].t_start_hr = 16;
    light[0].t_start_min = 0;
    light[0].t_stop_hr = 7;
    light[0].t_stop_min = 0;
    light[0].night_only = 0;

    //#1
    light[1].intensity_correction = 0;
    
    light[1].seq_list[0] = 30;    //B dash
    light[1].seq_list[1] = 10;
    light[1].seq_list[2] = 10;    //dot
    light[1].seq_list[3] = 10;
    light[1].seq_list[4] = 10;    // dot
    light[1].seq_list[5] = 30;     // char space
    light[1].seq_list[6] = 0;    
    light[1].seq_list[7] = 0;
    light[1].seq_list[8] = 0;
    light[1].seq_list[9] = 0;
    
    light[1].t_start_hr = 9;
    light[1].t_start_min = 0;
    light[1].t_stop_hr = 7;
    light[1].t_stop_min = 0;
    light[1].night_only = 0;
    
    //#2
    light[2].intensity_correction = 0;
    
    light[2].seq_list[0] = 2;     //C dash
    light[2].seq_list[1] = 10;
    light[2].seq_list[2] = 10;    // dot
    light[2].seq_list[3] = 10;
    light[2].seq_list[4] = 30;    // dash
    light[2].seq_list[5] = 10;
    light[2].seq_list[6] = 10;    // dot
    light[2].seq_list[7] = 30;    //char space
    light[2].seq_list[8] = 0;
    light[2].seq_list[9] = 0;
    
    light[2].t_start_hr = 0;
    light[2].t_start_min = 0;
    light[2].t_stop_hr = 7;
    light[2].t_stop_min = 0;
    light[2].night_only = 0;  

    //#3
    light[3].intensity_correction = 0;
    
    light[3].seq_list[0] = 30;   //D dash
    light[3].seq_list[1] = 10;
    light[3].seq_list[2] = 10;   //dot
    light[3].seq_list[3] = 10;
    light[3].seq_list[4] = 10;   //dot
    light[3].seq_list[5] = 30;
    light[3].seq_list[6] = 0;    
    light[3].seq_list[7] = 0;
    light[3].seq_list[8] = 0;
    light[3].seq_list[9] = 0;
    
    light[3].t_start_hr = 0;
    light[3].t_start_min = 0;
    light[3].t_stop_hr = 7;
    light[3].t_stop_min = 0;
    light[3].night_only = 0;  

    //#4
    light[4].intensity_correction = 0;
    
    light[4].seq_list[0] = 10;   //E dot
    light[4].seq_list[1] = 30;
    light[4].seq_list[2] = 0;
    light[4].seq_list[3] = 0;
    light[4].seq_list[4] = 0;
    light[4].seq_list[5] = 0;
    light[4].seq_list[6] = 0;    
    light[4].seq_list[7] = 0;
    light[4].seq_list[8] = 0;
    light[4].seq_list[9] = 0;
    
    light[4].t_start_hr = 0;
    light[4].t_start_min = 0;
    light[4].t_stop_hr = 7;
    light[4].t_stop_min = 0;
    light[4].night_only = 0; 

    //#5
    light[5].intensity_correction = 0;
    
    light[5].seq_list[0] = 10;   //F dot
    light[5].seq_list[1] = 10;
    light[5].seq_list[2] = 10;   //dot
    light[5].seq_list[3] = 10;
    light[5].seq_list[4] = 30;   //dash
    light[5].seq_list[5] = 10;
    light[5].seq_list[6] = 10;    //dot
    light[5].seq_list[7] = 30;
    light[5].seq_list[8] = 0;
    light[5].seq_list[9] = 0;
    
    light[5].t_start_hr = 0;
    light[5].t_start_min = 0;
    light[5].t_stop_hr = 7;
    light[5].t_stop_min = 0;
    light[5].night_only = 0; 

    //#6
    light[6].intensity_correction = 0;
    
    light[6].seq_list[0] = 30;   //G dash
    light[6].seq_list[1] = 10;
    light[6].seq_list[2] = 30;   //dash
    light[6].seq_list[3] = 10;
    light[6].seq_list[4] = 10;   //dot
    light[6].seq_list[5] = 30;
    light[6].seq_list[6] = 0; 
    light[6].seq_list[7] = 0;
    light[6].seq_list[8] = 0;
    light[6].seq_list[9] = 0;
    
    light[6].t_start_hr = 0;
    light[6].t_start_min = 0;
    light[6].t_stop_hr = 0;
    light[6].t_stop_min = 0;
    light[6].night_only = 0;   
    
    //#7
    light[7].intensity_correction = 0;
    
    light[7].seq_list[0] = 10;   //H dot
    light[7].seq_list[1] = 10;
    light[7].seq_list[2] = 10;   //dot
    light[7].seq_list[3] = 10;
    light[7].seq_list[4] = 10;   //dot
    light[7].seq_list[5] = 10;
    light[7].seq_list[6] = 10;    //dot
    light[7].seq_list[7] = 30;
    light[7].seq_list[8] = 0;
    light[7].seq_list[9] = 0;
    
    light[7].t_start_hr = 0;
    light[7].t_start_min = 0;
    light[7].t_stop_hr = 0;
    light[7].t_stop_min = 0;
    light[7].night_only = 0;     
    
    //#8
    light[8].intensity_correction = 0;
    
    light[8].seq_list[0] = 10;   //I dot
    light[8].seq_list[1] = 10;
    light[8].seq_list[2] = 10;   // dot
    light[8].seq_list[3] = 30;
    light[8].seq_list[4] = 0;
    light[8].seq_list[5] = 0;
    light[8].seq_list[6] = 0;    
    light[8].seq_list[7] = 0;
    light[8].seq_list[8] = 0;
    light[8].seq_list[9] = 0;
    
    light[8].t_start_hr = 16;
    light[8].t_start_min = 0;
    light[8].t_stop_hr = 7;
    light[8].t_stop_min = 0;
    light[8].night_only = 0; 

    //#9
    light[9].intensity_correction = 0;
    
    light[9].seq_list[0] = 10;   //J dot
    light[9].seq_list[1] = 10;
    light[9].seq_list[2] = 30;   //dash
    light[9].seq_list[3] = 10;
    light[9].seq_list[4] = 30;   //dash
    light[9].seq_list[5] = 10;
    light[9].seq_list[6] = 30;   //dash
    light[9].seq_list[7] = 30;
    light[9].seq_list[8] = 0;
    light[9].seq_list[9] = 0;
    
    light[9].t_start_hr = 0;
    light[9].t_start_min = 0;
    light[9].t_stop_hr = 7;
    light[9].t_stop_min = 0;
    light[9].night_only = 0;

    //#10
    light[10].intensity_correction = 0;
    
    light[10].seq_list[0] = 30;  //K dash
    light[10].seq_list[1] = 10;
    light[10].seq_list[2] = 10;  //dot
    light[10].seq_list[3] = 10;
    light[10].seq_list[4] = 30;  //dash
    light[10].seq_list[5] = 30;
    light[10].seq_list[6] = 0;    
    light[10].seq_list[7] = 0;
    light[10].seq_list[8] = 0;
    light[10].seq_list[9] = 0;
    
    light[10].t_start_hr = 16;
    light[10].t_start_min = 0;
    light[10].t_stop_hr = 7;
    light[10].t_stop_min = 0;
    light[10].night_only = 0;

    //#11
    light[11].intensity_correction = 0;
    
    light[11].seq_list[0] = 10;  //L dot
    light[11].seq_list[1] = 10;
    light[11].seq_list[2] = 30;  //dash
    light[11].seq_list[3] = 10;
    light[11].seq_list[4] = 10;  //dot
    light[11].seq_list[5] = 10;
    light[11].seq_list[6] = 10;    //dot
    light[11].seq_list[7] = 30;
    light[11].seq_list[8] = 0;
    light[11].seq_list[9] = 0;
    
    light[11].t_start_hr = 16;
    light[11].t_start_min = 0;
    light[11].t_stop_hr = 7;
    light[11].t_stop_min = 0;
    light[11].night_only = 0;

    // #12
    light[12].intensity_correction = 0;
    
    light[12].seq_list[0] = 30;  //M dash
    light[12].seq_list[1] = 10;
    light[12].seq_list[2] = 30;  //dash
    light[12].seq_list[3] = 30;
    light[12].seq_list[4] = 0;
    light[12].seq_list[5] = 0;
    light[12].seq_list[6] = 0;    
    light[12].seq_list[7] = 0;
    light[12].seq_list[8] = 0;
    light[12].seq_list[9] = 0;
    
    light[12].t_start_hr = 16;
    light[12].t_start_min = 0;
    light[12].t_stop_hr = 7;
    light[12].t_stop_min = 0;
    light[12].night_only = 0;

    //#13
    light[13].intensity_correction = 0;
    
    light[13].seq_list[0] = 30;  //N dash
    light[13].seq_list[1] = 10;
    light[13].seq_list[2] = 10;  //dot
    light[13].seq_list[3] = 30;
    light[13].seq_list[4] = 0;
    light[13].seq_list[5] = 0;
    light[13].seq_list[6] = 0;    
    light[13].seq_list[7] = 0;
    light[13].seq_list[8] = 0;
    light[13].seq_list[9] = 0;
    
    light[13].t_start_hr = 16;
    light[13].t_start_min = 0;
    light[13].t_stop_hr = 7;
    light[13].t_stop_min = 0;
    light[13].night_only = 0;
    
    //#14
    light[14].intensity_correction = 0;
    
    light[14].seq_list[0] = 30;  //O dash
    light[14].seq_list[1] = 10;
    light[14].seq_list[2] = 30;  //dash
    light[14].seq_list[3] = 10;
    light[14].seq_list[4] = 30;  //dash
    light[14].seq_list[5] = 30;
    light[14].seq_list[6] = 0;    
    light[14].seq_list[7] = 0;
    light[14].seq_list[8] = 0;
    light[14].seq_list[9] = 0;
    
    light[14].t_start_hr = 16;
    light[14].t_start_min = 0;
    light[14].t_stop_hr = 7;
    light[14].t_stop_min = 0;
    light[14].night_only = 0;  

    //#15
    light[15].intensity_correction = 0;
    
    light[15].seq_list[0] = 10;  //P dot
    light[15].seq_list[1] = 10;
    light[15].seq_list[2] = 30;  //dash
    light[15].seq_list[3] = 10;
    light[15].seq_list[4] = 30;  //dash
    light[15].seq_list[5] = 10;
    light[15].seq_list[6] = 10;    //dot
    light[15].seq_list[7] = 30;
    light[15].seq_list[8] = 0;
    light[15].seq_list[9] = 0;
    
    light[15].t_start_hr = 16;
    light[15].t_start_min = 0;
    light[15].t_stop_hr = 7;
    light[15].t_stop_min = 0;
    light[15].night_only = 0;  

    
    // TLC5940 #2 ---------------------------------------
    //#16
    light[16].intensity_correction = 0;
    
    light[16].seq_list[0] = 50;  //long iso-phase 
    light[16].seq_list[1] = 50;
    light[16].seq_list[2] = 0;
    light[16].seq_list[3] = 0;
    light[16].seq_list[4] = 0;
    light[16].seq_list[5] = 0;
    light[16].seq_list[6] = 0;    
    light[16].seq_list[7] = 0;
    light[16].seq_list[8] = 0;
    light[16].seq_list[9] = 0;
    
    light[16].t_start_hr = 16;
    light[16].t_start_min = 0;
    light[16].t_stop_hr = 7;
    light[16].t_stop_min = 0;
    light[16].night_only = 0; 

    //#17
    light[17].intensity_correction = 0;
    
    light[17].seq_list[0] = 5;  //flashing
    light[17].seq_list[1] = 20;
    light[17].seq_list[2] = 0;
    light[17].seq_list[3] = 0;
    light[17].seq_list[4] = 0;
    light[17].seq_list[5] = 0;
    light[17].seq_list[6] = 0;    
    light[17].seq_list[7] = 0;
    light[17].seq_list[8] = 0;
    light[17].seq_list[9] = 0;
    
    light[17].t_start_hr = 16;
    light[17].t_start_min = 0;
    light[17].t_stop_hr = 7;
    light[17].t_stop_min = 0;
    light[17].night_only = 0; 

    //#18
    light[18].intensity_correction = 0;
    
    light[18].seq_list[0] = 5;  //quick
    light[18].seq_list[1] = 5;
    light[18].seq_list[2] = 0;
    light[18].seq_list[3] = 0;
    light[18].seq_list[4] = 0;
    light[18].seq_list[5] = 0;
    light[18].seq_list[6] = 0;    
    light[18].seq_list[7] = 0;
    light[18].seq_list[8] = 0;
    light[18].seq_list[9] = 0;
    
    light[18].t_start_hr = 16;
    light[18].t_start_min = 0;
    light[18].t_stop_hr = 7;
    light[18].t_stop_min = 0;
    light[18].night_only = 0;   
    
    //#19
    light[19].intensity_correction = 0;
    
    light[19].seq_list[0] = 50;  // fixed on
    light[19].seq_list[1] = 0;
    light[19].seq_list[2] = 0;
    light[19].seq_list[3] = 0;
    light[19].seq_list[4] = 0;
    light[19].seq_list[5] = 0;
    light[19].seq_list[6] = 0;    
    light[19].seq_list[7] = 0;
    light[19].seq_list[8] = 0;
    light[19].seq_list[9] = 0;
    
    light[19].t_start_hr = 16;
    light[19].t_start_min = 0;
    light[19].t_stop_hr = 7;
    light[19].t_stop_min = 0;
    light[19].night_only = 0;     
    
    //#20
    light[20].intensity_correction = 0;
    
    light[20].seq_list[0] = 0;  // fixed off
    light[20].seq_list[1] = 0;
    light[20].seq_list[2] = 0;
    light[20].seq_list[3] = 0;
    light[20].seq_list[4] = 0;
    light[20].seq_list[5] = 0;
    light[20].seq_list[6] = 0;    
    light[20].seq_list[7] = 0;
    light[20].seq_list[8] = 0;
    light[20].seq_list[9] = 0;
    
    light[20].t_start_hr = 16;
    light[20].t_start_min = 0;
    light[20].t_stop_hr = 7;
    light[20].t_stop_min = 0;
    light[20].night_only = 0; 

    //#21
    light[21].intensity_correction = 0;
    
    light[21].seq_list[0] = 10;
    light[21].seq_list[1] = 20;
    light[21].seq_list[2] = 0;
    light[21].seq_list[3] = 0;
    light[21].seq_list[4] = 0;
    light[21].seq_list[5] = 0;
    light[21].seq_list[6] = 0;    
    light[21].seq_list[7] = 0;
    light[21].seq_list[8] = 0;
    light[21].seq_list[9] = 0;
    
    light[21].t_start_hr = 16;
    light[21].t_start_min = 0;
    light[21].t_stop_hr = 7;
    light[21].t_stop_min = 0;
    light[21].night_only = 0;

    //#22
    light[22].intensity_correction = 0;
    
    light[22].seq_list[0] = 10;
    light[22].seq_list[1] = 20;
    light[22].seq_list[2] = 0;
    light[22].seq_list[3] = 0;
    light[22].seq_list[4] = 0;
    light[22].seq_list[5] = 0;
    light[22].seq_list[6] = 0;    
    light[22].seq_list[7] = 0;
    light[22].seq_list[8] = 0;
    light[22].seq_list[9] = 0;
    
    light[22].t_start_hr = 16;
    light[22].t_start_min = 0;
    light[22].t_stop_hr = 7;
    light[22].t_stop_min = 0;
    light[22].night_only = 0;

    //#23
    light[23].intensity_correction = 0;
    
    light[23].seq_list[0] = 10;
    light[23].seq_list[1] = 20;
    light[23].seq_list[2] = 0;
    light[23].seq_list[3] = 0;
    light[23].seq_list[4] = 0;
    light[23].seq_list[5] = 0;
    light[23].seq_list[6] = 0;    
    light[23].seq_list[7] = 0;
    light[23].seq_list[8] = 0;
    light[23].seq_list[9] = 0;
    
    light[23].t_start_hr = 16;
    light[23].t_start_min = 0;
    light[23].t_stop_hr = 7;
    light[23].t_stop_min = 0;
    light[23].night_only = 0;

    //#24
    light[24].intensity_correction = 0;
    
    light[24].seq_list[0] = 10;
    light[24].seq_list[1] = 20;
    light[24].seq_list[2] = 0;
    light[24].seq_list[3] = 0;
    light[24].seq_list[4] = 0;
    light[24].seq_list[5] = 0;
    light[24].seq_list[6] = 0;    
    light[24].seq_list[7] = 0;
    light[24].seq_list[8] = 0;
    light[24].seq_list[9] = 0;
    
    light[24].t_start_hr = 16;
    light[24].t_start_min = 0;
    light[24].t_stop_hr = 7;
    light[24].t_stop_min = 0;
    light[24].night_only = 0;  

    //#25
    light[25].intensity_correction = 0;
    
    light[25].seq_list[0] = 10;
    light[25].seq_list[1] = 20;
    light[25].seq_list[2] = 0;
    light[25].seq_list[3] = 0;
    light[25].seq_list[4] = 0;
    light[25].seq_list[5] = 0;
    light[25].seq_list[6] = 0;    
    light[25].seq_list[7] = 0;
    light[25].seq_list[8] = 0;
    light[25].seq_list[9] = 0;
    
    light[25].t_start_hr = 16;
    light[25].t_start_min = 0;
    light[25].t_stop_hr = 7;
    light[25].t_stop_min = 0;
    light[25].night_only = 0;  

    //#26
    light[26].intensity_correction = 0;
    
    light[26].seq_list[0] = 10;
    light[26].seq_list[1] = 20;
    light[26].seq_list[2] = 0;
    light[26].seq_list[3] = 0;
    light[26].seq_list[4] = 0;
    light[26].seq_list[5] = 0;
    light[26].seq_list[6] = 0;    
    light[26].seq_list[7] = 0;
    light[26].seq_list[8] = 0;
    light[26].seq_list[9] = 0;
    
    light[26].t_start_hr = 16;
    light[26].t_start_min = 0;
    light[26].t_stop_hr = 7;
    light[26].t_stop_min = 0;
    light[26].night_only = 0; 

    //#27
    light[27].intensity_correction = 0;
    
    light[27].seq_list[0] = 10;
    light[27].seq_list[1] = 20;
    light[27].seq_list[2] = 0;
    light[27].seq_list[3] = 0;
    light[27].seq_list[4] = 0;
    light[27].seq_list[5] = 0;
    light[27].seq_list[6] = 0;    
    light[27].seq_list[7] = 0;
    light[27].seq_list[8] = 0;
    light[27].seq_list[9] = 0;
    
    light[27].t_start_hr = 16;
    light[27].t_start_min = 0;
    light[27].t_stop_hr = 7;
    light[27].t_stop_min = 0;
    light[27].night_only = 0; 

    //#28
    light[28].intensity_correction = 0;
    
    light[28].seq_list[0] = 10;
    light[28].seq_list[1] = 20;
    light[28].seq_list[2] = 0;
    light[28].seq_list[3] = 0;
    light[28].seq_list[4] = 0;
    light[28].seq_list[5] = 0;
    light[28].seq_list[6] = 0;    
    light[28].seq_list[7] = 0;
    light[28].seq_list[8] = 0;
    light[28].seq_list[9] = 0;
    
    light[28].t_start_hr = 16;
    light[28].t_start_min = 0;
    light[28].t_stop_hr = 7;
    light[28].t_stop_min = 0;
    light[28].night_only = 0;   
    
    //#29
    light[29].intensity_correction = 0;
    
    light[29].seq_list[0] = 10;
    light[29].seq_list[1] = 20;
    light[29].seq_list[2] = 0;
    light[29].seq_list[3] = 0;
    light[29].seq_list[4] = 0;
    light[29].seq_list[5] = 0;
    light[29].seq_list[6] = 0;    
    light[29].seq_list[7] = 0;
    light[29].seq_list[8] = 0;
    light[29].seq_list[9] = 0;
    
    light[29].t_start_hr = 16;
    light[29].t_start_min = 0;
    light[29].t_stop_hr = 7;
    light[29].t_stop_min = 0;
    light[29].night_only = 0;     
    
    //#30
    light[30].intensity_correction = 0;
    
    light[30].seq_list[0] = 10;
    light[30].seq_list[1] = 20;
    light[30].seq_list[2] = 0;
    light[30].seq_list[3] = 0;
    light[30].seq_list[4] = 0;
    light[30].seq_list[5] = 0;
    light[30].seq_list[6] = 0;    
    light[30].seq_list[7] = 0;
    light[30].seq_list[8] = 0;
    light[30].seq_list[9] = 0;
    
    light[30].t_start_hr = 16;
    light[30].t_start_min = 0;
    light[30].t_stop_hr = 7;
    light[30].t_stop_min = 0;
    light[30].night_only = 0; 

    //#31
    light[31].intensity_correction = 0;
    
    light[31].seq_list[0] = 10;
    light[31].seq_list[1] = 20;
    light[31].seq_list[2] = 0;
    light[31].seq_list[3] = 0;
    light[31].seq_list[4] = 0;
    light[31].seq_list[5] = 0;
    light[31].seq_list[6] = 0;    
    light[31].seq_list[7] = 0;
    light[31].seq_list[8] = 0;
    light[31].seq_list[9] = 0;
    
    light[31].t_start_hr = 16;
    light[31].t_start_min = 0;
    light[31].t_stop_hr = 7;
    light[31].t_stop_min = 0;
    light[31].night_only = 0;

    // end of light list    
}

/************************************
*                       *
************************************/
/*
Name: 
Synopsis: 
Requires:
Description: 
Author: Tony Cirineo
Date:  8/5/03
Revision History:
*/


// end


