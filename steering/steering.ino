#include <FastIO.h>
#include <I2CIO.h>
#include <LCD.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal_SR.h>
#include <LiquidCrystal_SR2W.h>
#include <LiquidCrystal_SR3W.h>

#include <PID_v1.h>
#include <mmc.h>
#include <avr/eeprom.h>
#include "file_store.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include <MPU6050.h>
//#include "mpu6050util.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Pin assignments
#define PIN_SERVO        9  // must be a PWM  
#define PIN_ROT_A        3 // INT1 
#define PIN_ROT_B        8 // potential to move this to A7
//#define PIN_ROT_PORT     PINB
#define PIN_ROT_PUSH     7 
#define PIN_ROT_LED_R    5
#define PIN_ROT_LED_G    6
#define PIN_ROT_LED_B    0 // no spare pins!!! - might be able to use 8


#define LCD_CHARS   16
#define LCD_LINES    2

// LCD
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// servo
#include <Servo.h> 
Servo rudder_servo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 

#define RUDDER_CENTRE 99 // the servo position that centres the rudder

// SD card logger
FileStore fileStore(640); // catalog (first) sector of data file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

//#define LED_PIN 13 
//long last_blink = 0;
bool led_on = false;

// MPU control/status vars
//bool dmpReady = false;  // set true if DMP init was successful
// expected DMP packet size (default is 42 bytes)
#define MPU_PACKET_SIZE 42
uint16_t fifoCount;     // count of all bytes currently in FIFO


// PID stuff

double target_yaw, current_yaw, target_rudder; // setpoint, input, output

// target_rudder is subject to quick changes which could stress or break the
// steering mechanism. current_rudder is a time dampened follower.
byte current_rudder;
byte rudder_rate = 30;  // notches per second - 180 = full range in 1 sec.
unsigned long rudder_time; // used by the rate limiter

//Define the default Tuning Parameters
// kp=0.5, ki=0.2, kd=0.1;
uint16_t EEMEM kp;
uint16_t EEMEM ki;
uint16_t EEMEM kd;
byte  EEMEM db;
PID myPID(&current_yaw, &target_rudder, &target_yaw, 1.0, 0.4, 0.2, REVERSE);

byte deadband = 1;  // default 1 degree
boolean autoSteer = false;
//boolean initialized = false;

// TODO move the following into a structure
// see http://books.google.co.uk/books?id=U6EtJwBzY1oC&amp;pg=PA100&amp;lpg=PA100&amp;dq=arduino+maximum+binary+sketch+size&amp;source=bl&amp;ots=EcziTPs6kG&amp;sig=N3RMoImznXxLGF3fwiNETKX-gBY&amp;hl=en&amp;sa=X&amp;ei=KDd-VOquKI3pasuUgfgO&amp;ved=0CEwQ6AEwBQ

// serial streaming support
//template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

long buttonTime = millis();
long rotaryTime = millis();
boolean isPressed = false;

typedef enum { E_NULL, E_ROT_INC, E_ROT_DEC, E_CLICK, E_LONG_CLICK } Event;
typedef struct FSM StateMachine; 
typedef void (*EventHandler)(StateMachine *sm, Event input);   
void checkButton();
void trigger(Event event);

struct FSM 
{ 
  EventHandler state; 
};

// Steer a fixed heading
void fsm_heading(StateMachine *fsm, Event event);
// Steer a fixed rudder angle
void fsm_rudder(StateMachine *fsm, Event event);
// Edit Kp (PID constant)
void fsm_kp(StateMachine *fsm, Event event);
// Edit Ki (PID constant)
void fsm_ki(StateMachine *fsm, Event event);
// Edit Kd (PID constant)
void fsm_kd(StateMachine *fsm, Event event);
// Edit Deadband (PID constant)
void fsm_db(StateMachine *fsm, Event event);
// Save PID constants
void fsm_save(StateMachine *fsm, Event event);
// Restore previously saved PID constants
void fsm_reset(StateMachine *fsm, Event event);

StateMachine ui = { fsm_rudder };

boolean eventRaised = true;

// GPS
#define GPS
#ifdef GPS

char gps_buffer[80];
int gps_index = 0;
char gps_line[] = "Waiting for GPS.";  // LCD display

static void process_gps_sentence(char *sentence);

#endif //GPS

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

volatile Event pendingEvent = E_NULL;
void decoder()
//very short interrupt routine 
//this routine is only called when pin1
//changes state, so it's the value of pin2 that we're
//interested in here
{
  unsigned long now = millis(); 
  if(now - rotaryTime > 5) { // 5 ms de-bounce period
    if (digitalRead(PIN_ROT_A) == digitalRead(PIN_ROT_B)) {
      //      trigger(E_ROT_INC); //if encoder channels are the same, direction is CW
      pendingEvent = E_ROT_INC;
    } else {
      //      trigger(E_ROT_DEC); //if they are not the same, direction is CCW
      pendingEvent = E_ROT_DEC;
    }
    rotaryTime = now;
  }
}

/*
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
*/

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    pinMode(PIN_ROT_PUSH, INPUT);  

    lcd.begin(LCD_CHARS, LCD_LINES);

    Serial.begin(9600); // speed of GPS NMEA

    fileStore.initialiseCatalog();

    // initialize device
    lcd.setCursor(0, 1);
    lcd.print(F("Init I2C..."));
    mpu.initialize();

    // verify connection
//    Serial.println(F("Testing device connections..."));
//    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
//    Serial.println(F("Initializing DMP..."));
    uint8_t devStatus = mpu.dmpInitialize();
//    uint8_t devStatus = dmpInitialize(mpu, fileStore);

    // gyro and accelerometer specific offsets - run MPU6050_calibration sketch to get these
    mpu.setXAccelOffset(-931);
    mpu.setYAccelOffset(-2082);
    mpu.setZAccelOffset(1374);
    mpu.setXGyroOffset(26);
    mpu.setYGyroOffset(-32);
    mpu.setZGyroOffset(-5);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(0, dmpDataReady, RISING);
//        uint8_t mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        lcd.setCursor(0, 1);
        lcd.print(F("DMP ready!"));
//        dmpReady = true;

        // get expected DMP packet size for later comparison
//        packetSize = mpu.dmpGetFIFOPacketSize();
//        Serial.print(F("Packet size .. "));
//        Serial.println(packetSize);
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        lcd.print(F("DMP Init failed: "));
        lcd.println(devStatus);
    }

    // enable interrupt for rotary encoder
    attachInterrupt(1, decoder, LOW);
    pinMode(PIN_ROT_PUSH, INPUT); // sets analog pin for input 
    pinMode(PIN_ROT_A, INPUT_PULLUP);  
    pinMode(PIN_ROT_B, INPUT_PULLUP); // sets analog pin for input 

    // initialise PID algorithm
    myPID.SetOutputLimits(0, 180);
    
    // read PID parameters from EEPROM
    uint16_t eeprom_kp = eeprom_read_word(&kp);
    uint16_t eeprom_ki = eeprom_read_word(&ki);
    uint16_t eeprom_kd = eeprom_read_word(&kd);
    deadband = eeprom_read_byte(&db);
    myPID.SetTunings(eeprom_kp, eeprom_ki, eeprom_kd);

    // initialise servo
    rudder_servo.attach(PIN_SERVO);
    target_rudder = RUDDER_CENTRE;  // servo in centre position
    current_rudder = RUDDER_CENTRE;
    rudder_time = millis();
    
    fileStore.logSeparator();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    if(pendingEvent != E_NULL) {
      Event event = pendingEvent;
      pendingEvent = E_NULL;
      trigger(event);      
    }

    checkButton();

#ifdef GPS
    // check GPS
    if(Serial.available()) {
      do {
        char aChar = Serial.read();
        if((aChar == '$' && gps_index > 0) || gps_index >= 78) {
          // End of command.
//      Serial.println(gps_buffer);
          process_gps_sentence(gps_buffer);
          memset(gps_buffer, 0, 80);
          gps_index = 1;
          gps_buffer[0] = '$';
//          gps_buffer[1] = NULL;
        } else {
          gps_buffer[gps_index] = aChar;
          gps_index++;
          gps_buffer[gps_index] = '\0'; // Keep the string NULL terminated
 //     Serial.println(gps_buffer);
        }
      } while(Serial.available());
    }
#endif //GPS

    // wait for MPU interrupt or extra packet(s) available
    if(mpuInterrupt || fifoCount >= MPU_PACKET_SIZE) {
  
      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      uint8_t mpuIntStatus = mpu.getIntStatus();
  
      // get current FIFO count
      fifoCount = mpu.getFIFOCount();
      
      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
          // reset so we can continue cleanly
          mpu.resetFIFO();
//          Serial.println(F("FIFO overflow!"));
  
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & 0x02) {
           // wait for correct available data length, should be a VERY short wait
          while (fifoCount < MPU_PACKET_SIZE) fifoCount = mpu.getFIFOCount();
  
          int32_t qw, qx, qy, qz;  // quaternion
          { // limit scope of fifoBuffer
            uint8_t fifoBuffer[MPU_PACKET_SIZE]; // FIFO storage buffer
            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, MPU_PACKET_SIZE);
          
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= MPU_PACKET_SIZE;
  
            // get quaternion from fifoBuffer
            qw = ((fifoBuffer[0] << 8) + fifoBuffer[1]);
            qx = ((fifoBuffer[4] << 8) + fifoBuffer[5]);
            qy = ((fifoBuffer[8] << 8) + fifoBuffer[9]);
            qz = ((fifoBuffer[12] << 8) + fifoBuffer[13]);
            current_yaw = atan2((float)(qx*qy - qw*qz),
                                (float)(qw*qw + qx*qx - 134217728l))  * 57.29578;
          } // end of scope of fifoBuffer (memory released)

          // update the PID
          // before ivoking the compute method, we need to guard against 0/360 degree
          // wrap around confusing the PID and trying to drive the servo the wrong way
          if (current_yaw - target_yaw < -180)
             current_yaw += 360;
          if (current_yaw - target_yaw > 180)
             current_yaw -= 360;

          long current_time = millis();
//          fileStore.logImu(current_time, current_yaw, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI); // yaw, pitch, roll
          fileStore.logQuaternion(current_time, (int16_t)qw, (int16_t)qx, (int16_t)qy, (int16_t)qz);

          // dead-band - turn off PID if error is below threshold
          if(autoSteer) {
            boolean inDeadband = (abs(target_yaw - current_yaw) < deadband);
            if(inDeadband) {
              myPID.SetMode(MANUAL);
            } else if(myPID.GetMode() == MANUAL) {
              myPID.SetMode(AUTOMATIC);
            }
          } else if(myPID.GetMode() == AUTOMATIC) {
              myPID.SetMode(MANUAL);
          }

          if(myPID.Compute()) {
            // new output available
            refresh_display();
            
            // log to SD card
            fileStore.logPid(current_time, target_yaw, current_yaw, target_rudder);
          }
      }
    }

    // adjust the rudder - every 50 ms
    unsigned long now = millis();
    unsigned long last_adjustment = now - rudder_time;
    if((last_adjustment) >= 50) {
      double rudder_diff = target_rudder - (double)current_rudder;
      double shift_limit = (rudder_rate * (double)last_adjustment / 1000.0);
      byte new_position;
      if(abs(rudder_diff) > shift_limit) {
        new_position = current_rudder + copysign(shift_limit, rudder_diff);
      } else {
        new_position = target_rudder;
      }
      if(new_position != current_rudder) {
        current_rudder = new_position;
        rudder_servo.write(current_rudder);
      }
      refresh_display();
      rudder_time = now;
    }
    
}

void refresh_display() {
  char line[17];
 
   // two screens, depending on state
  if(ui.state == fsm_heading || ui.state == fsm_rudder) {
    // screen 1 - steering to fixed heading

    // line 1 - GPS output
    lcd.setCursor(0,0);
    lcd.print(gps_line);

    // line 2 - steering status
    strcpy(line, "                ");

    // heading
    int disp_yaw = (int)target_yaw % 360;
    while(disp_yaw < 0) disp_yaw += 360;
    format_number(disp_yaw, 0, line+3);
    line[3] = 0xDF;  // degrees

    if(ui.state == fsm_heading) {
      // error - how well it's steering to heading
      int disp_error = (int)abs(target_yaw - current_yaw);
      format_number(disp_error, 0, line+8);
      line[8] = 0xDF;  // degrees
      if(current_yaw > target_yaw) {
        line[5] = 0x7F; // left arrow
      } else {
        line[9] = 0x7E; // right arrow
      }
    } else {
      // fixed rudder - no error
      line[5] = 'F'; line[6] = 'i'; line[7] = 'x'; line[8] = 'e'; line[9] = 'd';
    }

    // rudder
    int disp_rudder = (int)(abs(target_rudder - RUDDER_CENTRE) * 1.1111);
    format_number(disp_rudder, 0, line+14);
    line[14] = '%';
    if(target_rudder < RUDDER_CENTRE) {
      line[11] = 0x7F;
    } else {
      line[15] = 0x7E;
    }

    lcd.setCursor(0, 1);
    lcd.print(line);          
  } else {
    // screen 2 - PID setup
    // line 1
    lcd.setCursor(0,0);
    strcpy(line, "Kp=     Ki=     ");
    format_number(myPID.GetKp() / 10, 1, line+7);
    format_number(myPID.GetKi() / 10, 1, line+15);

    if(ui.state == fsm_kp) {
      line[2] = 0x7E; // right arrow
      line[7] = 0x7F; // left arrow
    } else if(ui.state == fsm_ki) {
      line[10] = 0x7E; // right arrow
      line[15] = 0x7F; // left arrow
    }
    lcd.print(line);

    // line 2 
    lcd.setCursor(0,1);
    strcpy(line, "Kd=     Db=   S ");
    format_number(myPID.GetKd() / 10, 1, line+7);
    format_number(deadband, 0, line+13);

    if(ui.state == fsm_reset) {
      line[14] = 'R';
    }
    
    if(ui.state == fsm_kd) {
      line[2] = 0x7E; // right arrow
      line[7] = 0x7F; // left arrow
    } else if(ui.state == fsm_db) {
      line[10] = 0x7E; // right arrow
      line[13] = 0x7F; // left arrow
    } else if(ui.state == fsm_save || ui.state == fsm_reset) {
      line[13] = 0x7E; // right arrow
      line[15] = 0x7F; // left arrow
    }

    lcd.print(line);
  }
}

char *format_number(unsigned x, char shift, char *s) {
//    *--s = 0;
  if(shift) {
    if (!x) *--s = '0';
    for (; x && shift--; x /= 10) {
      *--s = '0' + x % 10;
    }
    *--s = '.';
  } 
  if (!x) *--s = '0';
  for (; x; x /= 10) {
    *--s = '0' + x % 10;
  }
  return s;
}

#ifdef GPS
static void process_gps_sentence(char *sentence) {
//   Serial << "GPS: "<<  sentence << "\n"; 
  // only care about $GPRMC sentences
  /*
  $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A

Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
     *6A          The checksum data, always begins with *
   */
   
  char *ptr;
  char *saveptr;
  char *timeptr;
  if(strstr(sentence, "$GPRMC")) {
    // time
    ptr = strtok_r(sentence+7, ",", &saveptr);
    timeptr = ptr;
    
    // active or void?
    ptr = strtok_r(NULL, ",", &saveptr);
    // TODO if void, abandon and return immediately
    if(*ptr == 'V') {
      gps_line[15] = 'V';
      return;
    }
    // latitude
    ptr = strtok_r(NULL, ",", &saveptr);
    double latitude = atof(ptr+2) * 0.0166667; // divide by 60
    *(ptr+2) = '\0';
    latitude += atoi(ptr);
    ptr = strtok_r(NULL, ",", &saveptr);
    if(*ptr == 'S') {
      latitude *= -1.0;
    } // otherwise N - positive lat

    // longitude
    ptr = strtok_r(NULL, ",", &saveptr);
    double longitude = atof(ptr+3) * 0.0166667; // divide by 60
    *(ptr+2) = '\0';
    longitude += atoi(ptr);
    ptr = strtok_r(NULL, ",", &saveptr);
    if(*ptr == 'W') {
      longitude *= -1.0;
    } // otherwise E - positive long

    // speed *10
    ptr = strtok_r(NULL, ",", &saveptr);
    int speed_knots = (int)(10.0 * atof(ptr));

    // heading *10   
    ptr = strtok_r(NULL, ",", &saveptr);
    int track_angle = (int)(10.0 * atof(ptr));

    // date
    ptr = strtok_r(NULL, ",", &saveptr);
    // combine with the time
    strncpy(sentence, ptr, 6);
    strncpy(sentence+6, timeptr, 6);
    // this needs null terminating
    sentence[12] = '\0';
  
//    Serial << "***GPS: "<<  sentence << ", " << latitude << ", " << longitude << ", " << speed_knots << ", " << track_angle << "\n"; 
    // log GPS data
    fileStore.logGps(millis(), sentence, latitude, longitude, speed_knots, track_angle);
    // set the lcd line
    strcpy(gps_line, "    kn         A");
    format_number(speed_knots, 1, gps_line+4);
    format_number(track_angle, 1, gps_line+13);
    gps_line[13] = 0xDF;  // degrees
    
    refresh_display();
  }
}
#endif //GPS

void checkButton() {
 // ignore anything that happens within 5 ms of last activity (de-bounce)
  boolean buttonPushed = (digitalRead(PIN_ROT_PUSH) == HIGH);
  if(buttonTime == 0) {
    if(!buttonPushed) {  // button finally release after LongClick
      isPressed = false;
      buttonTime = millis();
    }
    return;
  }
  if(millis() - buttonTime > 5) {  // de-bounce time - 5ms
    if(!isPressed) {  // currently not pressed
      if(buttonPushed) {  // button now pressed
        isPressed = true;
        buttonTime = millis();
      }
    } else {  // currently pressed
      if(buttonPushed && millis() - buttonTime > 1000) {
        // *** FIRE LongClick event
        trigger(E_LONG_CLICK);
        buttonTime = 0; // signals the need to wait until button is released
      }
      if(!buttonPushed) {
        // *** FIRE Click event
        trigger(E_CLICK);
        isPressed = false;
        buttonTime = millis();
      }
    }
  }
}

void trigger(Event event) {
  (ui.state)(&ui, event);
  eventRaised = true;
}

void fsm_heading(StateMachine *fsm, Event event) {
  switch(event) {
    case E_ROT_INC:
      target_yaw++;
      break;
    case E_ROT_DEC:
      target_yaw--;
      break;
    case E_CLICK:
      autoSteer = false; // turn off PID control
      fsm->state = fsm_rudder;
      break;
    case E_LONG_CLICK:
      fsm->state = fsm_kp;
      break;
  }
}

void fsm_rudder(StateMachine *fsm, Event event) {
  switch(event) {
    case E_ROT_INC:
      if(target_rudder >= 1)
        target_rudder--;
      break;
    case E_ROT_DEC:
      if(target_rudder <= 179)
        target_rudder++;
      break;
    case E_CLICK:
      // switch to auto-heading using the current_heading
      target_yaw = current_yaw;
      autoSteer = true; // turn on PID control
      fsm->state = fsm_heading;
      break;
    case E_LONG_CLICK:
      // centre the rudder (slowly)
      target_rudder = RUDDER_CENTRE;
      break;
  }
}

void fsm_kp(StateMachine *fsm, Event event) {
  switch(event) {
    case E_ROT_INC:
      myPID.SetTunings(myPID.GetKp() + 10, myPID.GetKi(), myPID.GetKd());
      break;
    case E_ROT_DEC:
      if(myPID.GetKp() > 0)
        myPID.SetTunings(myPID.GetKp() - 10, myPID.GetKi(), myPID.GetKd());
      break;
    case E_CLICK:
      fsm->state = fsm_ki;
      break;
    case E_LONG_CLICK:
      fsm->state = fsm_save;
      break;
  }
}

void fsm_ki(StateMachine *fsm, Event event) {
  switch(event) {
    case E_ROT_INC:
      myPID.SetTunings(myPID.GetKp(), myPID.GetKi() + 10, myPID.GetKd());
      break;
    case E_ROT_DEC:
      if(myPID.GetKi() > 0)
        myPID.SetTunings(myPID.GetKp(), myPID.GetKi() - 10, myPID.GetKd());
      break;
    case E_CLICK:
      fsm->state = fsm_kd;
      break;
    case E_LONG_CLICK:
      fsm->state = fsm_save;
      break;
  }
}

void fsm_kd(StateMachine *fsm, Event event) {
  switch(event) {
    case E_ROT_INC:
      myPID.SetTunings(myPID.GetKp(), myPID.GetKi(), myPID.GetKd() + 10);
      break;
    case E_ROT_DEC:
      if(myPID.GetKd() > 0)
      myPID.SetTunings(myPID.GetKp(), myPID.GetKi(), myPID.GetKd() - 10);
      break;
    case E_CLICK:
      fsm->state = fsm_db;
      break;
    case E_LONG_CLICK:
      fsm->state = fsm_save;
      break;
  }
}

void fsm_db(StateMachine *fsm, Event event) {
  switch(event) {
    case E_ROT_INC:
      deadband++;
      break;
    case E_ROT_DEC:
      deadband--;
      break;
    case E_CLICK:
      fsm->state = fsm_kp;
      break;
    case E_LONG_CLICK:
      fsm->state = fsm_save;
      break;
  }
}

void fsm_save(StateMachine *fsm, Event event) {
  switch(event) {
    case E_ROT_INC:
      fsm->state = fsm_reset;
      break;
    case E_ROT_DEC:
      fsm->state = fsm_reset;
      break;
    case E_CLICK:
      // save the values to EEPROM
      eeprom_write_word(&kp, myPID.GetKp());
      eeprom_write_word(&ki, myPID.GetKi());
      eeprom_write_word(&kd, myPID.GetKd());
      eeprom_write_byte(&db, deadband);
      fsm->state = fsm_heading;
      break;
    case E_LONG_CLICK:
      fsm->state = fsm_kp;
      break;
  }
}

void fsm_reset(StateMachine *fsm, Event event) {
  switch(event) {
    case E_ROT_INC:
      fsm->state = fsm_save;
      break;
    case E_ROT_DEC:
      fsm->state = fsm_save;
      break;
    case E_CLICK:
    {
      // restore PID parameters from EEPROM
      uint16_t eeprom_kp = eeprom_read_word(&kp);
      uint16_t eeprom_ki = eeprom_read_word(&ki);
      uint16_t eeprom_kd = eeprom_read_word(&kd);
      deadband = eeprom_read_byte(&db);
      myPID.SetTunings(eeprom_kp, eeprom_ki, eeprom_kd);
      
      fsm->state = fsm_heading;
      break;
    }
    case E_LONG_CLICK:
      fsm->state = fsm_kp;
      break;
  }
}

