//############### SEESTAR CAMERA SYSTEM LICENSE ################################
//#
//#  This work is distributed under a Creative Commons Attribution-ShareAlike 4.0 license.
//#  Derivative works should be shared under the same license:
//#    http://creativecommons.org/licenses/by-sa/4.0
//#
//#  See http://bitbucket.org/mbari/seestar for contact information and source
//#
//#  Citation: F. Cazenave, C. Kecy, M. Risi, S.H.D. Haddock (in press)
//#     "SeeStar: a low-cost, modular, and open-source camera system for subsea observations",
//#     IEEE Oceans 2014
//#
//#  Social Note: Thom Maughan is just a bit player (coder and board bringup)
//#               in this project
//#
//##############################################################################

// SeeStar v2 TimeLapse Arduino Controller Code for GoPro Hero3/3+
// Author: Thom Maughan
// Date:   7 Aug 2015
// Version: 0.8 (Alpha)
// Requires: seestar2_lib

// Features:
// Works with Arduino Pro Mini AT328 3.3v 8MHz
// Uses MBARI open source hardware shield for GoPro<>Arduio hardware interface
// DS3234 low drift Real Time Clock
// Low Power Sleep with RTC wakeup interrupt
// Uses GoPro interface connnector
// LED Flashlight

// Status of Code:  NOT WORKING YET
// 1915.08.17 16:46:06 should print 2015
// setting alarm1 for number of seconds interval is in progress - reprogramming alarm, now need to generalize to sec,min,hour depending on the length of the interval
// need to add video duration
// voltage and current(read_battery) have wrong scaling - was working for maker, not sure what changed (hardware?)
// serial command processing and serial command definition
// every other time Arduino upload is done, the code does not run (BIG PROBLEM)
// Need to work out LPM code - serial wakeup, RTC wakeup
// Need to test serial without Arduino/codebender - thru serial terminal
// Need to calculate interval and programming next RTC wakeup



#include <avr/pgmspace.h>    // needed from PROGMEM strings to save SRAM
#include <SPI.h>
#include <atmega328pwr.h>    // seestar2_lib
#include <ds3234.h>          // seestar2_lib
#ifdef SERCMDON
#include <SerCmd.h>  // seestar2_lib
#endif

// Digital IO symbolic definitions (All are Outputs except those with 'IN' comments)
#define ARD_LED        13    // same as SCK (conflicts with DS3234 SPI bus)
#define RTC_INT_L      2     // IN  //WAKEUP_PIN = 2;
#define LIGHT_BUTTON   4     // was PIN4
#define RTC_RESET_L    5
#define UART_SHDN_L    6
#define LIGHT_POWER    7
#define CAM_ON         8
#define CAM_POWER      9
#define RTC_CS_L       10

// Analog INPUTs
#define BAT_VOLT    A0
#define BAT_CURR    A1

#ifdef NOCODE
const char hlp_cmd_0[] PROGMEM = "HELP:         help menu";
const char prm_cmd_1[] PROGMEM = "PARAM:        get parameters";
const char tim_cmd_2[] PROGMEM = "TIME:         get time";
const char stm_cmd_3[] PROGMEM = "SETTIME:      set time";
const char int_cmd_4[] PROGMEM = "INTERVAL:     Set timelapse interval, time between taking image/video";
const char dur_cmd_5[] PROGMEM = "DURATION:     Duration of video in seconds";
const char dls_cmd_6[] PROGMEM = "DELAYSTART:   Set Delay in seconds after START commmand before starting timelapse";
const char srt_cmd_7[] PROGMEM = "START:        Start timelapse";    // was CamGo
const char gbt_cmd_8[] PROGMEM = "GETBAT:       Get battery voltage and current";    
const char ldl_cmd_9[] PROGMEM = "LEDDELAY:     Set LED delay in milli seconds";
const char ldr_cmd_10[] PROGMEM = "LEDDURATION: Set LED on time in milli seconds";

const char ga1_cmd_11[] PROGMEM = "GetAlarm1: ";
const char ga2_cmd_12[] PROGMEM = "GetAlarm2: ";
const char sa1_cmd_13[] PROGMEM = "SetAlarm1: ";
const char sa2_cmd_14[] PROGMEM = "SetAlarm2: ";
const char gtp_cmd_15[] PROGMEM = "GetTemp: ";

const char lon_cmd_16[] PROGMEM = "LEDON: ";
const char lof_cmd_17[] PROGMEM = "LEDOFF: ";

const char irt_cmd_18[] PROGMEM = "InitRTC: ";
const char gst_cmd_19[] PROGMEM = "GetStat: ";
const char rag_cmd_20[] PROGMEM = "ResetAge: ";
const char cmd_21[] PROGMEM = "       ";
const char cmd_22[] PROGMEM = "       ";
const char cmd_23[] PROGMEM = "       ";
const char cmd_24[] PROGMEM = "       ";
const char cmd_25[] PROGMEM = "       ";
const char cmd_26[] PROGMEM = "       ";
const char cmd_27[] PROGMEM = "       ";
const char cmd_28[] PROGMEM = "       ";
const char cmd_29[] PROGMEM = "       ";


// Set up a table to refer to the strings.

const char* const cmd_table1[] PROGMEM = {
        hlp_cmd_0, prm_cmd_1, tim_cmd_2, stm_cmd_3, int_cmd_4, 
        dur_cmd_5, dls_cmd_6, srt_cmd_7, gbt_cmd_8, ldl_cmd_9, 
        ldr_cmd_10, ga1_cmd_11, ga2_cmd_12, sa1_cmd_13, sa2_cmd_14, 
        gtp_cmd_15, lon_cmd_16, lof_cmd_17, irt_cmd_18, gst_cmd_19,
        rag_cmd_20, cmd_21, cmd_22, cmd_23, cmd_24,
        cmd_25, cmd_26, cmd_27, cmd_28, cmd_29
};
#endif

// const
//const char menuCmd[][2] PROGMEM = 
//  { 
//    "MENU", "help menu",
//    "GetAlarm1", "get DS3234 alarm1",
//    "GetAlarm2", "get DS3234 alarm2",
//    "SetAlarm1", "set DS3234 alarm1 ss.mm.hh.dd",
//    "SetAlarm2", "set DS3234 alarm2 mm.hh.dd",
//    "GetTemp", "get DS3234 temperature",
//    "ResetAlarm", "reset DS3234 status reg alarm flags",
//    "ResetAge", "reset DS3234 aging register",
//    "GetStat", "get DS3234 status register",
//    "InitRTC", "init DS3234 rtc",
//    "TIME", "get time",
//    "LEDON", "led light on",
//    "LEDOFF", "led light off",
//    "GetBat", "get battery voltage and current",
//    "Interval", "set timelapse interval, how often to take image/video",
//    "Duration", "set video duration, how long to record",
//    "CamGo", "start timelapse"
//  };





// global variable declarations



int batCurrent = 0;
int batVoltage = 0;

// serial port variables
//#define RX_BUF_SIZE 	256
//#define RX_BUF_SIZE 	8
//char rxBuf[RX_BUF_SIZE];

#define PRN_BUF_SIZE	80
char prnBuf[PRN_BUF_SIZE];

#ifdef  SERCMDON
//SerCmd SCmd;   // The SerCmd object
#endif

#define SRAM_SIZE 256		// size of SRAM in DS3234

// time variables

uint8_t time[8];
uint8_t sleep_period = 5;       // the sleep interval in minutes between 2 consecutive alarms
struct ts t;
  
unsigned int rxCnt = 0;
unsigned long prev;


// Intervelometer stuff

//unsigned long interval = 5000;		// milliseconds
unsigned long interval = 10000;		// milli-seconds, interval between photos / videos

unsigned int alarmSec = 0;			// for keeping track of programming
unsigned int alarmInterval = 10;	// 10 seconds


//do Not adjust MIN_INTERVAL (this is deterimined by how long it takes to power up and take an image and power down
#define MIN_INTERVAL  25000          // 25seconds

// MakerDemo: adjust INTERVAL to determine intervelometer time between photos (30000 = 30 sec)
#define INTERVAL      30000          // msec between pictures, NO less than MIN_INTERVAL

// interrupt service routine
volatile int rtc_intVal = 0;		// volatile since value is set in DS3224_wake interrupt service routine


void setup()
{
  // put your setup code here, to run once:

  //pinMode(ARD_LED, OUTPUT);      //  = 13;  (port 13 is also SPI bus SCK
  pinMode(RTC_INT_L, INPUT);       //  = 2;
  pinMode(RTC_RESET_L, OUTPUT);    //  = 5;
  pinMode(UART_SHDN_L, OUTPUT);    //  = 6;
  pinMode(LIGHT_POWER, OUTPUT);    //  = 7;
  pinMode(CAM_ON, OUTPUT);       //  = 8;
  pinMode(CAM_POWER, OUTPUT);    //  = 9;
  pinMode(RTC_CS_L, OUTPUT);       //  = 10;

  // Note: dont use Arduino LED (pin 13) and DS34322 RTC
  //digitalWrite(ARD_LED, LOW);      // LED off
  digitalWrite(RTC_RESET_L, HIGH);  // active lo uart reset
  digitalWrite(UART_SHDN_L, LOW);  // active lo uart reset
  digitalWrite(LIGHT_POWER, LOW);     // active hi
  digitalWrite(CAM_ON, LOW);        // active hi
  digitalWrite(CAM_POWER, LOW);     // active hi
  digitalWrite(RTC_CS_L, HIGH);     // active lo



  Serial.begin(9600);


  // Setup callbacks for SerCmd commands, pass index of cmd_table
  // it's a bit inconvenient, but the indx needs to match the string
  // order in the cmd_table above (side effect of going to PROGMEM)
#ifdef  SERCMDON  
  SCmd.addCommand(0, get_menu);
  SCmd.addCommand(1, get_param);
  SCmd.addCommand(2, get_time);
  SCmd.addCommand(3, set_time);
  SCmd.addCommand(4, set_interval);
  SCmd.addCommand(5, set_duration);
  SCmd.addCommand(6, set_delaystart);
  SCmd.addCommand(7, cam_go);  
  SCmd.addCommand(8, get_battery);
  SCmd.addCommand(9, set_led_delay);
  SCmd.addCommand(10, set_led_duration);  
  SCmd.addCommand(11, get_DS3234_alarm1);
  SCmd.addCommand(12, get_DS3234_alarm2);
  SCmd.addCommand(13, set_DS3234_alarm1);
  SCmd.addCommand(14, set_DS3234_alarm2);
  SCmd.addCommand(15, get_DS3234_temperature);
  SCmd.addCommand(16, led_light_on);  
  SCmd.addCommand(17, led_light_off); 
  SCmd.addCommand(18, init_rtc); 
  SCmd.addCommand(19, get_DS3234_status_reg); 
  SCmd.addCommand(20, reset_DS3234_aging_reg); 
#endif    
  
  
  
  
  //NOTE: Max command size is 12 (see SerCmd.h)
  //SCmd.addCommand("Howdy", Howdy1);
  //SCmd.addCommand("MENU", get_menu);
  //SCmd.addCommand("GetAlarm1", get_DS3234_alarm1);
  //SCmd.addCommand("GetAlarm2", get_DS3234_alarm2);
  //SCmd.addCommand("SetAlarm1", set_DS3234_alarm1);
  //SCmd.addCommand("SetAlarm2", set_DS3234_alarm2);
  
  //SCmd.addCommand("GetAgingReg", get_DS3234_aging_reg);
  //SCmd.addCommand("GetSRAM", get_DS3234_sram);
  //SCmd.addCommand("GetTemp", get_DS3234_temperature);
  //SCmd.addCommand("ResetAlarm", reset_DS3234_status_reg_alarm);
  //SCmd.addCommand("ResetAge", reset_DS3234_aging_reg);
  //SCmd.addCommand("GetStat", get_DS3234_status_reg);
  //SCmd.addCommand("InitRTC", init_rtc);
  //SCmd.addCommand("TIME", get_time);



  //SCmd.addCommand("LEDON", led_light_on);
  //SCmd.addCommand("LEDOFF", led_light_off);
  //SCmd.addCommand("GetBat", print_battery);

  //SCmd.addCommand("Interval", cam_interval);
  //SCmd.addCommand("Duration", cam_duration);
  //SCmd.addCommand("CamGo", cam_go);


  //SCmd.addCommand("P", process_command); // Converts two arguments to integers and echos them back
  //SCmd.addDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?")

  // use ds3234 library

// OLD PRELIB
      //RTC_init();   // Note: dont use Arduino LED (pin 13) and RTC
      //day(1-31), month(1-12), year(0-99), hour(0-23), minute(0-59), second(0-59)
      //SetTimeDate(17,8,15,15,34,55);

//#define JUSTARD 1
#ifdef JUSTARD
  // seestar_lib DS3234 init
  DS3234_init(RTC_CS_L, DS3234_INTCN);    // RTC_CS_L = 10


  /* Now it is time to enable an interrupt. In the function call
     * attachInterrupt(A, B, C)
     * A   can be either 0 or 1 for interrupts on pin 2 or 3.
     *
     * B   Name of a function you want to execute while in interrupt A.
     *
     * C   Trigger mode of the interrupt pin. can be:
     *             LOW        a low level trigger
     *             CHANGE     a change in level trigger
     *             RISING     a rising edge of a level trigger
     *             FALLING    a falling edge of a level trigger
     *
     * In all but the IDLE sleep modes only LOW can be used.
     */


  //digitalWrite (RTC_INT_L, HIGH);  // internal pull-up resistor (likely not needed, but what the heck)

  attachInterrupt (0, ds3234_wake, LOW);  // attach interrupt handler for RTC_INT wakeup (interrupt when low)
  //attachInterrupt (0, ds3234_wake, CHANGE);  // attach interrupt handler for RTC_INT wakeup (interrupt when low)
  //attachInterrupt (0, ds3234_wake, FALLING);  // attach interrupt handler for RTC_INT wakeup (interrupt when low)


  // reserve 200 bytes for the inputString:
  //inputString.reserve(200);

  alarmInterval = 10;
  alarmSec = 0;
  set_alarm1_sec(alarmSec);
#endif
  //set_alarm_1hz();
  while(!Serial);
  Serial.println("Setup is Complete");



  //Serial.println("TssmmhhWDDMMYYYY aka set time");

}


void get_menu(void)
{
  int i;
// for(i=0; i<SCmd.numCommand; i++)  // defd in SerCmd.h
  for(i=0; i<20; i++)  
  {
    
    strcpy_P(prnBuf, (char*)pgm_read_word(&(cmd_table1[i]))); // Necessary casts and dereferencing, just copy.
    Serial.println(prnBuf);
    
//    Serial.println( SCmd.getCmd(i) );
//    Serial.println( menuCmd[i][0]);
  }
}


void loop()
{
  uint8_t reg_val;
  unsigned long now = millis();

#ifdef SERCMDON
  // process serial commands
  SCmd.readSerial();     //
#endif

  if (rtc_intVal == 1)
  {

    //alarmInterval = 10;  hardcoded alarm interval
    alarmSec += alarmInterval;
    if (alarmSec >= 60)
    {
      alarmSec = alarmSec % 60;
    }
    set_alarm1_sec(alarmSec);  // program alarm1 for next wakeup

    Serial.println("RTC interrupt happened");
    rtc_intVal = 0;			// isr variable from ds3234_wake

    //intervelometer();

    // sleep the processor, wake on serial or RTC interrupt
    //if(Serial.available() <= 0)
    //{
    //	Sleepy::powerDown ();
    //	Sleepy::loseSomeTime(10000);		// sleep for one second or unless woke by RTC
    //}
  }
#ifdef JUSTARD
  // show time once in a while
  if ((now - prev > interval) && (Serial.available() <= 0))
  {
    DS3234_get(RTC_CS_L, &t);
    snprintf(prnBuf, PRN_BUF_SIZE, "%d.%02d.%02d %02d:%02d:%02d", t.year,
             t.mon, t.mday, t.hour, t.min, t.sec);
    Serial.println(prnBuf);
    prev = now;

  }
#endif

}

void Howdy1()
{
  Serial.println("Howdy1 back at ya");

}

void Howdy2()
{
  Serial.println("Howdy2 back at ya");

}

int read_battery(int prnFlg)
{
  unsigned long ulBat;
  unsigned int voltCnt, currCnt;
  float batVolt;

  voltCnt = (unsigned int) analogRead(A0);
  currCnt = (unsigned int) analogRead(A1);

  ulBat = (unsigned long) voltCnt;
  ulBat *= 3184;              // 504 cnts = 16.05v implies 31.85 scale factor
  ulBat /= 100;
  batVoltage = (unsigned int) ulBat;

  batCurrent = currCnt * 49;    // scale to milliamps (approx) (4.88 mA / cnt)
  batCurrent /= 10;            // 4.9

  if (prnFlg == 1)
  {
    Serial.print("Voltage = ");
    batVolt = (float)batVoltage;
    batVolt /= 1000;
    Serial.print(batVolt);
    Serial.print(" V ");
    //Serial.print(voltCnt);    // uncomment to check calibration
    //Serial.print(" cnt ");

    Serial.print("  Current = ");
    Serial.print(batCurrent);
    Serial.print(" mA ");
    //Serial.print(currCnt);
    //Serial.print(" cnt");
    Serial.println(" ");
  }

}



void get_time (void)
{
    DS3234_get(RTC_CS_L, &t);
    snprintf(prnBuf, PRN_BUF_SIZE, "%d.%02d.%02d %02d:%02d:%02d", t.year,
             t.mon, t.mday, t.hour, t.min, t.sec);
    Serial.println(prnBuf);  
}
void init_rtc (void)
{
  DS3234_init(RTC_CS_L, DS3234_INTCN);
}




// Active-Low Interrupt or Square-Wave Output. This open-drain pin requires an external pullup resistor. It can be
// left open if not used. This multifunction pin is determined by the state of the INTCN bit in the Control Register
// (0Eh). When INTCN is set to logic 0, this pin outputs a square wave and its frequency is determined by RS2
// and RS1 bits. When INTCN is set to logic 1, then a match between the timekeeping registers and either of the
// alarm registers activates the INT // /SQW pin (if the alarm is enabled). Because the INTCN bit is set to logic 1
// when power is first applied, the pin defaults to an interrupt output with alarms disabled. The pullup voltage can
// be up to 5.5V, regardless of the voltage on VCC. If not used, this pin can be left unconnected.

// 1 per second sq wave
void set_alarm_1hz(void)
{
  DS3234_set_creg(RTC_CS_L, 0);

}

// set the alarm time (hour,min,sec)
void set_alarm1_XXX(void)
{

  // flags define what calendar component to be checked against the current time in order
  // to trigger the alarm - see datasheet
  // A1M1 (seconds) (0 to enable, 1 to disable)
  // A1M2 (minutes) (0 to enable, 1 to disable)
  // A1M3 (hour)    (0 to enable, 1 to disable)
  // A1M4 (day)     (0 to enable, 1 to disable)
  // DY/DT          (dayofweek == 1/dayofmonth == 0)
  uint8_t flags[5] = { 0, 0, 0, 1, 1 };

  // time when to wake up
  uint8_t wake_HOUR = 15;
  uint8_t wake_MINUTE = 46;
  uint8_t wake_SECOND = 9;

  // set Alarm1
  DS3234_set_a1(RTC_CS_L, wake_SECOND, wake_MINUTE, wake_HOUR, 0, flags);

  // activate Alarm1
  //DS3234_set_creg(cs, DS3234_INTCN | DS3234_A1IE);
  //DS3234_set_creg(RTC_CS_L, 0);
  DS3234_set_creg(RTC_CS_L, DS3234_INTCN | DS3234_A1IE);


}

// set the alarm time (hour,min,sec)
void set_alarm1_sec(unsigned int seconds)
{
  // use alarm1

  //	if(seconds > 59)
  //	{
  //		seconds = 59;
  //	}
  //
  //	seconds = 40;

  //Serial.print("Seconds ");
  //Serial.println(DEC, seconds);

  //seconds = 10;		// DEBUG

  // flags define what calendar component to be checked against the current time in order
  // to trigger the alarm - see datasheet
  // A1M1 (seconds) (0 to enable, 1 to disable)
  // A1M2 (minutes) (0 to enable, 1 to disable)
  // A1M3 (hour)    (0 to enable, 1 to disable)
  // A1M4 (day)     (0 to enable, 1 to disable)
  // DY/DT          (dayofweek == 1/dayofmonth == 0)
  uint8_t flags[5] = { 0, 1, 1, 1, 1 };

  // time when to wake up
  //uint8_t wake_HOUR = 15;
  //uint8_t wake_MINUTE = 46;
  //uint8_t wake_SECOND = 30;

  // set Alarm1
  //DS3234_set_a1(RTC_CS_L, wake_SECOND, 0, 0, 0, flags);
  DS3234_set_a1(RTC_CS_L, seconds, 0, 0, 0, flags);

  // activate Alarm1
  //DS3234_set_creg(cs, DS3234_INTCN | DS3234_A1IE);
  //DS3234_set_creg(RTC_CS_L, 0);
  DS3234_set_creg(RTC_CS_L, DS3234_INTCN | DS3234_A1IE);

}


// set alarm minutes (not hour,day) using DS3234 alarm 2
void set_next_alarm2(void)
{
  struct ts t;
  unsigned char wakeup_min;

  DS3234_get(RTC_CS_L, &t);

  // calculate the minute when the next alarm will be triggered
  wakeup_min = (t.min / sleep_period + 1) * sleep_period;
  if (wakeup_min > 59)
  {
    wakeup_min -= 60;
  }

  // flags define what calendar component to be checked against the current time in order
  // to trigger the alarm
  // A2M2 (minutes) (0 to enable, 1 to disable)
  // A2M3 (hour)    (0 to enable, 1 to disable)
  // A2M4 (day)     (0 to enable, 1 to disable)
  // DY/DT          (dayofweek == 1/dayofmonth == 0)
  uint8_t flags[4] = { 0, 1, 1, 1 };

  // set Alarm2. only the minute is set since we ignore the hour and day component
  DS3234_set_a2(RTC_CS_L, wakeup_min, 0, 0, flags);

  // activate Alarm2
  DS3234_set_creg(RTC_CS_L, DS3234_INTCN | DS3234_A2IE);
}

// Interrupt Service Routine (ISR) for DS3234 RTC interrupt (active low)
void ds3234_wake(void)
//void pinChange ()
{
  uint8_t reg_val;

  //DS3234_clear_a1f(RTC_INT_L);			// clear the interrupt by writing to the DS3234 status reg

  reg_val = DS3234_get_sreg(RTC_CS_L);
  reg_val &= B11111100;
  DS3234_set_sreg(RTC_CS_L, reg_val);

  rtc_intVal = 1;		// signal foreground that interrupt occurred
  //if (digitalRead (RTC_INT_L) == HIGH)
  //digitalWrite (LIGHT_POWER, HIGH);
  //else
  //digitalWrite (LIGHT_POWER, LOW);
}  // end of ds3234_wake


void process_command()
{
  int aNumber;
  char *arg;

  Serial.println("We're in process_command");
#ifdef SERCMDON  
  arg = SCmd.next('.');
  if (arg != NULL)
  {
    aNumber = atoi(arg);  // Converts a char string to an integer
    Serial.print("First argument was: ");
    Serial.println(aNumber);
  }
  else {
    Serial.println("No arguments");
  }

  arg = SCmd.next();
  if (arg != NULL)
  {
    aNumber = atol(arg);
    Serial.print("Second argument was: ");
    Serial.println(aNumber);
  }
  else {
    Serial.println("No second argument");
  }
#endif
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized()
{
  Serial.println("What?");
}




#ifdef  NOCODE
//void serial_rx(void)

// serialEvent is called by Arduino after loop()
void serialEventXXX(void)
{
  int in;

  if (Serial.available() > 0)
  {
    in = Serial.read();

    if ((in == 10 || in == 13) && (rxCnt > 0)) // if carriage return / linefeed
    {
      parse_cmd(rxBuf, rxCnt);
      rxCnt = 0;
      rxBuf[0] = 0;
    }
    else if (in < '0' || in > 'z')    // if not valid character
    { // ~[0-9A-Za-z]
      // ignore
      //Serial.println('x');
    }
    else if (rxCnt > RX_BUF_SIZE - 2)
    {
      // drop
      Serial.println('buf overflow, drop');
      rxCnt = 0;
      rxBuf[0] = 0;
    }
    else if (rxCnt < RX_BUF_SIZE - 2)
    {
      rxBuf[rxCnt] = in;
      rxBuf[rxCnt + 1] = 0;
      rxCnt += 1;
    }

  }   // if serial available

}
#endif

void set_time (void)
{
  Serial.println("set time ...");
}

void set_delaystart (void)
{
  Serial.println("set delay start in sec ...");
}

void set_led_delay (void)
{
  Serial.println("set led delay ...");
}

void set_led_duration (void)
{
  Serial.println("set led duration ...");
}

void get_param (void)
{
  Serial.println("get param is ...");
}

void set_interval (void)
{
  // set the interval, how often to take picture or shoot video

  Serial.println("Cam Interval is: NOT WORKING YET");
}


void set_duration (void)
{
  // set the duration (applies to video, ignored for image timelapse)
  // duration must be less than interval, it will be automatically reduced

  Serial.println("Cam Duration is: NOT WORKING YET");


}


void cam_go (void)
{
  // take picture or shoot video
  Serial.println("Cam Go is: NOT WORKING YET");

}


void get_DS3234_alarm1(void)
{
  DS3234_get_a1(RTC_CS_L, &prnBuf[0], 59);
  Serial.println(prnBuf);
}

void get_DS3234_alarm2(void)
{
  DS3234_get_a2(RTC_CS_L, &prnBuf[0], 59);
  Serial.println(prnBuf);
}

void get_DS3234_aging_reg(void)
{
  Serial.print("aging reg is ");
  Serial.println(DS3234_get_aging(RTC_CS_L), DEC);
}

void get_DS3234_sram(void)
{
  int i;
  char sramByte;
  for (i = 0; i < SRAM_SIZE - 1; i++)
  {
    sramByte = DS3234_get_sram_8b(RTC_CS_L, i);

    Serial.print(sramByte, DEC);
    Serial.print(" ");

  }
}

//Set Alarm1 SSMMHHDD
void set_DS3234_alarm1(void)
{
  char *arg;

#ifdef  SERCMDON
  arg = SCmd.next();
#endif  
  if (arg != NULL)
  {
    //aNumber=atoi(arg);    // Converts a char string to an integer
    Serial.print("First argument: ");
    Serial.println(arg);
  }
  else
  {
    Serial.println("No SSMMHHDD arg");
  }

  DS3234_set_creg(RTC_CS_L, DS3234_INTCN | DS3234_A1IE);
  //SSMMHHDD
  //for (i = 0; i < 4; i++)
  //{
  //    time[i] = (cmd[2 * i + 1] - 48) * 10 + cmd[2 * i + 2] - 48; // ss, mm, hh, dd
  //}
  // SS.MM.HH.DD
  time[0] = (arg[0] & 0x0f) * 10 + (arg[1] & 0x0f);
  time[1] = (arg[3] & 0x0f) * 10 + (arg[4] & 0x0f);
  time[2] = (arg[6] & 0x0f) * 10 + (arg[7] & 0x0f);
  time[3] = (arg[9] & 0x0f) * 10 + (arg[10] & 0x0f);

  uint8_t flags[5] = { 0, 0, 0, 0, 0 };
  DS3234_set_a1(RTC_CS_L, time[0], time[1], time[2], time[3], flags);
  DS3234_get_a1(RTC_CS_L, &prnBuf[0], 59);
  Serial.println(prnBuf);
}

void set_DS3234_alarm2(void)
{
  char *arg;

#ifdef SERCMDON
  arg = SCmd.next();
#endif  
  if (arg != NULL)
  {
    //aNumber=atoi(arg);    // Converts a char string to an integer
    Serial.print("First argument: ");
    Serial.println(arg);
  }
  else
  {
    Serial.println("No SS.MM.HH arg");
  }


  DS3234_set_creg(RTC_CS_L, DS3234_INTCN | DS3234_A2IE);
  // SS.MM.HH
  time[0] = (arg[0] & 0x0f) * 10 + (arg[1] & 0x0f);
  time[1] = (arg[3] & 0x0f) * 10 + (arg[4] & 0x0f);
  time[2] = (arg[6] & 0x0f) * 10 + (arg[7] & 0x0f);


  uint8_t flags[5] = { 0, 0, 0, 0, 0 };
  DS3234_set_a2(RTC_CS_L, time[0], time[1], time[2], flags);
  DS3234_get_a2(RTC_CS_L, &prnBuf[0], 59);
  Serial.println(prnBuf);

  //Serial.println("B - set DS3234 Alarm 2, format mm.hh.dd, ex. B15.09.10 is 10th day at 9:15 am");
}


void get_DS3234_temperature (void)
{
  Serial.print("temperature reg is ");
  Serial.println(DS3234_get_treg(RTC_CS_L), DEC);

  //Serial.println("C - get DS3234 temperature register");

}

void reset_DS3234_status_reg_alarm (void)
{
  uint8_t reg_val;

  // "D" - reset status register alarm flags
  reg_val = DS3234_get_sreg(RTC_CS_L);
  reg_val &= B11111100;
  DS3234_set_sreg(RTC_CS_L, reg_val);
  Serial.print("reset status reg alarm flags");

  //Serial.println("D - reset DS3234 status register alarm flags");

}


void reset_DS3234_aging_reg (void)
{
  // "G" - reset aging status register
  DS3234_set_aging(RTC_CS_L, 0);
}


void led_light_on (void)
{
  unsigned int dur;

  // turn power on to the LED flashlight - this does not turn on the light, this enables the LED flashlight controller
  Serial.println("Light Power ON");
  digitalWrite(LIGHT_POWER, HIGH);

  read_battery(1);
  delay(500);

  // press the LED flashlight button
  Serial.println("Light Button Push");
  digitalWrite(LIGHT_BUTTON, HIGH);   // was PIN4
  delay(200);
  digitalWrite(LIGHT_BUTTON, LOW);

  //read_battery(1);
  delay(100);
  read_battery(1);


  // Turn off the LED Flashlight
  //Serial.println("Light Power OFF");
  //digitalWrite(LIGHT_POWER, LOW);

  //read_battery(1);
}

void led_light_off (void)
{
  unsigned int dur;

  read_battery(1);

  // Turn off the LED Flashlight
  Serial.println("Light Power OFF");
  digitalWrite(LIGHT_POWER, LOW);

  delay(100);

  read_battery(1);
}

void get_battery (void)
{
  read_battery(1);
}

void get_DS3234_status_reg (void)
{
  Serial.print("status reg is ");
  Serial.println(DS3234_get_sreg(RTC_CS_L), DEC);

  //Serial.println("S - get DS3234 status register");
}


void set_DS3234_time (void)
{
  char *arg;
  struct ts t;
  // TODO, get from arg
  //T355720619112011

#ifdef SERCMDON
  //35.57.20.6.19.11.2011
  arg = SCmd.next();
#endif  

  t.sec = inp2toi(arg, 1);
  t.min = inp2toi(arg, 3);
  t.hour = inp2toi(arg, 5);
  t.wday = inp2toi(arg, 7);
  t.mday = inp2toi(arg, 8);
  t.mon = inp2toi(arg, 10);
  t.year = inp2toi(arg, 12) * 100 + inp2toi(arg, 14);
  DS3234_set(RTC_CS_L, t);
  Serial.println("OK");

  // not the correct cmdsize
  //Serial.println("T - set DS3234 Time, format: TssmmhhWDDMMYYYY  ex. T015509319082015");
}

String ReadTimeDate()
{
  String temp;
  int TimeDate [7]; //second,minute,hour,null,day,month,year

  for (int i = 0; i <= 6; i++)
  {
    if (i == 3)
      i++;
    digitalWrite(RTC_CS_L, LOW);
    SPI.transfer(i + 0x00);
    unsigned int n = SPI.transfer(0x00);
    digitalWrite(RTC_CS_L, HIGH);
    int a = n & B00001111;
    if (i == 2)
    {
      int b = (n & B00110000) >> 4; //24 hour mode
      if (b == B00000010)
        b = 20;
      else if (b == B00000001)
        b = 10;
      TimeDate[i] = a + b;
    }
    else if (i == 4)
    {
      int b = (n & B00110000) >> 4;
      TimeDate[i] = a + b * 10;
    }
    else if (i == 5)
    {
      int b = (n & B00010000) >> 4;
      TimeDate[i] = a + b * 10;
    }
    else if (i == 6)
    {
      int b = (n & B11110000) >> 4;
      TimeDate[i] = a + b * 10;
    }
    else
    {
      int b = (n & B01110000) >> 4;
      TimeDate[i] = a + b * 10;
    }
  }
  temp.concat(TimeDate[4]);
  temp.concat("/") ;
  temp.concat(TimeDate[5]);
  temp.concat("/") ;
  temp.concat(TimeDate[6]);
  temp.concat("     ") ;
  temp.concat(TimeDate[2]);
  temp.concat(":") ;
  temp.concat(TimeDate[1]);
  temp.concat(":") ;
  temp.concat(TimeDate[0]);
  return (temp);
}

void intervelometer()   // was loop for maker demo
{


  unsigned int msec;
  unsigned int duration;

  Serial.print(((float) (INTERVAL)) / 1000);
  Serial.println(" sec interval");

  Serial.println(ReadTimeDate());

  duration = powerup_take_image_powerdown();

  Serial.print(duration);
  Serial.println(" msec duration");

  Serial.println(ReadTimeDate());

  read_battery(1);    // 1 => print

  if (INTERVAL >= duration)
  {
    msec = INTERVAL - duration;
  }
  else
  {
    msec = MIN_INTERVAL - duration; // minimum interval is 10 sec
  }

  //delay(msec);
  //Serial.print(msec);
  //Serial.println(" msec loop delay");
}


//CAMPOWER: _|--------------------------------------------------------------------------------------------------------|__
//CAM_ON:   ____||_______________________________________________________________________________________________________
//LEDPOWER: ____|-----------------------------------------------------------------------|________________________________
//LED_ON:   ______________________________________________________________||_____________________________________________
//Take Pix: __________________________________________________________________________||_________________________________
//Seconds:   0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19   20   21

// on the root of the GoPro uSD is autoexec.ash, it should look like this (with Unix line terminations)
//################################################
//# Hero3+: Silver: wait 5, PhotoMode, wait 5, Take one image, wait 5 seconds (for image write?) and power down #
//################################################
//
//sleep 5
//t app appmode photo
//sleep 5
//t app button shutter PR
//sleep 5
//poweroff yes
//reboot yes


// autoexec.ash delays (useed in the defines below)
#define ASH_BOOTDELAY      5000
#define ASH_MENUDELAY      5000   // delay after photomode and before shutter 
#define ASH_TAKEPHOTODELAY 5000

// definitions for the camera power and light power sequence / timeline
#define CAM_POWERUP_DELAY  1000   // was 1000
#define CAM_ON_BUTTON      100
#define LIGHT_ON_DELAY     2000   // delay after shutter to turn on Light
#define CAM_ON_TO_LIGHT_ON ASH_BOOTDELAY + ASH_MENUDELAY + LIGHT_ON_DELAY  // 12000  // ash script delays before PR are here: 5000ash + 5000ash + 2000
#define LIGHT_ON_BUTTON    200    // button press duration 100 does not work
#define LIGHT_ON_DURATION  3000   // 3000
#define CAM_OFF_DELAY      6000



unsigned int powerup_take_image_powerdown()
{
  unsigned int i;
  unsigned int dur;
  unsigned int msec = 0;
  int prnBatFlg;
  unsigned long startTime;
  unsigned long deltaTime;

  prnBatFlg = 1;    // print battery readings, 1 says print

  startTime = millis();
  Serial.println(startTime);

  deltaTime = millis() - startTime;
  Serial.println(deltaTime);

  read_battery(prnBatFlg);

  deltaTime = millis() - startTime;
  Serial.println(deltaTime);

  // turn on the 16v input power to the CamDo battery eliminator in the GoPro
  Serial.println("Camera Power ON");
  digitalWrite(CAM_POWER, HIGH);

  // powerup delay for camera
  dur = CAM_POWERUP_DELAY;
  Serial.print(dur);
  Serial.println(" msec");
  delay(dur);
  msec += dur;

  read_battery(prnBatFlg);

  deltaTime = millis() - startTime;
  Serial.println(deltaTime);

  // turn power on to the LED flashlight - this does not turn on the light, this enables the LED flashlight controller
  Serial.println("Light Power ON");
  digitalWrite(LIGHT_POWER, HIGH);

  // Push the GoPro camera 'on' button (pin 12 on GoPro bus)
  dur = CAM_ON_BUTTON;
  Serial.println("Take Picture");
  digitalWrite(CAM_ON, HIGH);
  delay(dur);          // small 'button push' delay (TODO: find out how small)
  digitalWrite(CAM_ON, LOW);
  msec += dur;

  // Delay between GoPro Camera ON button push and turning on LED flashlight (ash script delays + scene adjust alg)
  // LED flash light should come on with a couple seconds to allow GoPro auto white balance algorithm time to adjust
  dur = CAM_ON_TO_LIGHT_ON;
  Serial.print(dur);
  Serial.println(" msec");
  delay(dur);
  read_battery(prnBatFlg);
  msec += dur;

  deltaTime = millis() - startTime;
  Serial.println(deltaTime);
  read_battery(prnBatFlg);


  // press the LED flashlight button
  dur = LIGHT_ON_BUTTON;
  Serial.println("Light Button Push");
  digitalWrite(LIGHT_BUTTON, HIGH);   //was PIN4
  delay(dur);
  digitalWrite(LIGHT_BUTTON, LOW);
  msec += dur;



  read_battery(prnBatFlg);

  // light on duration, light should turn off just after GoPro takes picture (photo num increments in GoPro display)
  dur = LIGHT_ON_DURATION;
  Serial.print(dur);
  Serial.println(" msec");
  delay(dur);
  msec += dur;

  read_battery(prnBatFlg);

  deltaTime = millis() - startTime;
  Serial.println(deltaTime);

  // Turn off the LED Flashlight
  Serial.println("Light Power OFF");
  digitalWrite(LIGHT_POWER, LOW);

  read_battery(prnBatFlg);

  // wait a period of time to insure image is written to uSD card
  dur = CAM_OFF_DELAY;
  Serial.print(dur);
  Serial.println(" msec");
  for (i = 0; i < 10; i++)
  {
    delay(dur / 10);
    msec += dur / 10;
    read_battery(prnBatFlg);
    if (batCurrent < 80)   // batCurrent is 60mA to 80mA typically after GoPro shuts itself down
    {
      // a little more delay just to be safe
      delay(dur / 10);
      msec += dur / 10;
      Serial.println("GoPro powered off");
      //break;   // break and GoPro will be powered off based on current, break commmented for safe delay
    }
  }

  deltaTime = millis() - startTime;
  Serial.println(deltaTime);

  Serial.println("Camera Power OFF");
  digitalWrite(CAM_POWER, LOW);

  return (msec);
}


// same routine as above with debug statements removed
unsigned int simple_powerup_take_image_powerdown()
{
  unsigned int dur;
  unsigned int msec = 0;

  // turn on the 16v input power to the CamDo battery eliminator in the GoPro
  digitalWrite(CAM_POWER, HIGH);

  // powerup delay for GoPro / Battery Eliminator
  dur = CAM_POWERUP_DELAY;
  delay(dur);
  msec += dur;

  // turn power on to the LED flashlight
  digitalWrite(LIGHT_POWER, HIGH);

  // Push the GoPro camera 'on' button (pin 12 on GoPro bus)
  dur = CAM_ON_BUTTON;
  digitalWrite(CAM_ON, HIGH);
  delay(dur);          // 'button push' delay
  digitalWrite(CAM_ON, LOW);
  msec += dur;

  // Delay between GoPro Camera ON button push and turning on LED flashlight (ash script delays + scene adjust alg)
  dur = CAM_ON_TO_LIGHT_ON;
  delay(dur);
  msec += dur;

  // press the LED flashlight button
  dur = LIGHT_ON_BUTTON;
  digitalWrite(LIGHT_BUTTON, HIGH);   // was PIN4
  delay(dur);
  digitalWrite(LIGHT_BUTTON, LOW);
  msec += dur;

  // light on duration, light should turn off just after GoPro takes picture (photo num increments in GoPro display)
  dur = LIGHT_ON_DURATION;
  delay(dur);
  msec += dur;

  // Turn off the LED Flashlight
  digitalWrite(LIGHT_POWER, LOW);

  // wait a period of time to insure image is written to uSD card
  dur = CAM_OFF_DELAY;
  delay(dur);

  digitalWrite(CAM_POWER, LOW);

  return (msec);
}








// use watchdog for timed wakeup
ISR(WDT_vect)
{
 // Sleepy::watchdogEvent();  // Setup for low power waiting
}




// for pin mapping on AT168/AT328, see:
// http://arduino.cc/en/uploads/Hacking/Atmega168PinMap2.png?random=1428344290232

// Arduino Pro Mini Bus, Starting at BLK on FTDI
//  D1   TX       31|                    |     RAW
//  D0   RX       30|                    |     GND
//  RESET         29|                    |29   RESET
//                  |                    |     VCC
//  D2   INT0     32|                    |26   ADC3
//  D3   INT1/PWM  1|                    |25   ADC2
//  D4             2|                    |24   ADC1
//  D5   PWM       9|                    |23   ADC0
//  D6   PWM/AIN0 10|                    |17   D13 SCK   ARD_LED
//  D7   AIN1     11|                    |15   D12 MOSI
//  D8            12|                    |16   D11 MISI
//  D9            13|                    |14   D10 SS



//PARAM - display current user parameters
//DEFAULT - set user parameters to default
//SAVE - save user parameters to flash
//-----TIME - display RTC time
//HELP - display help menu
//----LED ON - toggle the LED relay on
//----LED OFF - toggle the LED relay off
//SAMPLE NOW - take a sample now
//GET LED - get the LED relay state
//GET POWER - get the power consumption
//SET TIME - set the RTC time
//SET AUTO RUN MODE - set the auto run mode [ON|OFF]
//SET AUTO RUN DELAY - se tthe auto run delay (SEC)
//
//SET AUTO RUN DURATION - set the auto run duration (SEC)
//SET SAMPLE MODE - set the sample mode [OFF PIC VID HR3]
//SET SAMPLE INTERVAL - set the interval of the sample (SEC)
//SET SAMPLE DURATION - set the duration of the sample (SEC)
//SET LED ON DELAY - set the LED relay delay (HR3 only)
//SET LED ON DURATION - set the LED duration (HR3 only)
//
//-------- LOW LEVEL COMMANDS --------
//EEPROG - program the on board EEPROM
//EEREAD - read the on board EEPROM
//CAM ON - turn on the GoPro via GP_MODE
//CAM OFF - turn off the GoPro via GP_MODE
//SNAPHI - execute snap hi I/O sequence
//SNAPLO - execute the snap lo I/O sequence
//VID ON - set GP_IO2 lo to start the video recording
//VID OFF - set GP_ID2 hi to stop video recording
//PWR ON - apply power to the GoPro
//PWR OFF - remove power from the GoPro
//GET DEBUG - display which modules have debuging enabled
//SET DEBUG - enalbe debugging for a module
//CLEAR DEBUG - disable module debugging
//
//RDY

//help all
//Remove  RESET


//PARAM - display current user parameters
//DEFAULT - set user parameters to default
//SAVE - save user parameters to flash
//TIME - display RTC time
//HELP - display help menu
//LED ON - toggle the LED relay on
//LED OFF - toggle the LED relay off
//SAMPLE NOW - take a sample now
//GET LED - get the LED relay state
//GET POWER - get the power consumption
//SET TIME - set the RTC time
//SET AUTO RUN MODE - set the auto run mode [ON|OFF]
//SET AUTO RUN DELAY - se tthe auto run delay (SEC)
//
//SET AUTO RUN DURATION - set the auto run duration (SEC)
//SET SAMPLE MODE - set the sample mode [OFF PIC VID HR3]
//SET SAMPLE INTERVAL - set the interval of the sample (SEC)
//SET SAMPLE DURATION - set the duration of the sample (SEC)
//SET LED ON DELAY - set the LED relay delay (HR3 only)
//SET LED ON DURATION - set the LED duration (HR3 only)
//
//-------- LOW LEVEL COMMANDS --------
//EEPROG - program the on board EEPROM
//EEREAD - read the on board EEPROM
//CAM ON - turn on the GoPro via GP_MODE
//CAM OFF - turn off the GoPro via GP_MODE
//SNAPHI - execute snap hi I/O sequence
//SNAPLO - execute the snap lo I/O sequence
//VID ON - set GP_IO2 lo to start the video recording
//VID OFF - set GP_ID2 hi to stop video recording
//PWR ON - apply power to the GoPro
//PWR OFF - remove power from the GoPro
//GET DEBUG - display which modules have debuging enabled
//SET DEBUG - enalbe debugging for a module
//CLEAR DEBUG - disable module debugging
//
//RDY
