// Pin assignments
#define PIN_SERVO        9  // must be a PWM  
#define PIN_LCD_RS       4  
#define PIN_LCD_EN       7
#define PIN_LCD_D4      14
#define PIN_LCD_D5      15
#define PIN_LCD_D6      16
#define PIN_LCD_D7      17
#define PIN_ROT_A        3 // INT1
#define PIN_ROT_B        8//A7
#define PIN_ROT_PUSH    A6 // using analog input - set internal pullup resistor
#define PIN_ROT_LED_R    5 // PWM
#define PIN_ROT_LED_G    6 // PWM
#define PIN_ROT_LED_B    0 // no spare pins - use 8 if we can move PIN_ROT_B onto A7 (analog input only)


#define LCD_CHARS   16
#define LCD_LINES    2

float target_yaw=0;
uint16_t kp=50, ki=20, kd=10; // these are 100x their real values
byte db=0;

long buttonTime = millis();
long rotaryTime = millis();
boolean isPressed = false;

typedef enum { E_ROT_INC, E_ROT_DEC, E_CLICK, E_LONG_CLICK } Event;
typedef struct FSM StateMachine; 
typedef void (*EventHandler)(StateMachine *sm, Event input);   
void checkButton();
void trigger(Event event);

struct FSM 
{ 
  EventHandler state; 
};

void fsm_manual(StateMachine *fsm, Event event);
void fsm_kp(StateMachine *fsm, Event event);
void fsm_ki(StateMachine *fsm, Event event);
void fsm_kd(StateMachine *fsm, Event event);
void fsm_db(StateMachine *fsm, Event event);
void fsm_save(StateMachine *fsm, Event event);
void fsm_reset(StateMachine *fsm, Event event);

StateMachine ui = { fsm_manual };

boolean eventRaised = true;

// LCD
#include <LiquidCrystal.h>
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

void setup() {
  // put your setup code here, to run once:
    pinMode(PIN_ROT_PUSH, INPUT); // sets analog pin for input 
    pinMode(PIN_ROT_A, INPUT_PULLUP);  
    pinMode(PIN_ROT_B, INPUT_PULLUP); // sets analog pin for input 
    attachInterrupt(1, decoder, LOW);
    lcd.begin(LCD_CHARS, LCD_LINES);
  Serial.begin(9600);

    digitalWrite(PIN_ROT_LED_R, HIGH);
}

void loop() {
  checkButton();
  
  if(eventRaised) {
    eventRaised = false;

    char gps_line[] = "3.6kn 256       ";  // test data
    gps_line[9] = 0xDF;  // degrees
    
    char line[17];
  
    // two screens, depending on state
    if(ui.state == fsm_manual) {
      // screen 1 - steering to fixed heading

      // line 1 - GPS output
      lcd.setCursor(0,0);
      lcd.print(gps_line);

      // line 2 - steering status
      
      float current_yaw = 10; // sample
      float rudder_angle = 75; // sample
      
//      char frag[6];
      
      int disp_yaw = (int)target_yaw % 360;
      while(disp_yaw < 0) disp_yaw += 360;
      
//      dtostrf(disp_yaw, 3, 0, frag);
//      strcpy(line, frag);
//      strcat(line, "   ");
      
      int disp_error = (int)abs(target_yaw - current_yaw);
//      dtostrf(disp_error, 2, 0, frag);
//      strcpy(line+6, frag);
//      strcat(line, "    ");

      int disp_rudder = (int)(abs(rudder_angle - 90) * 1.1111);
//      dtostrf(disp_rudder, 2, 0, frag);
//      strcpy(line+12, frag);
//      strcat(line, "% ");

      memset(line, ' ', 16);
      format_number(disp_yaw, 0, line+3);
      format_number(disp_error, 0, line+8);
      format_number(disp_rudder, 0, line+14);
      line[14] = '%';
    
//      sprintf(line, "%03d   %02d    %02d%% ", disp_yaw, disp_error, disp_rudder);

      line[3] = 0xDF;  // degrees
      line[8] = 0xDF;  // degrees
      if(current_yaw > target_yaw) {
        line[5] = 0x7F; // left arrow
      } else {
        line[9] = 0x7E; // right arrow
      }
      if(rudder_angle < 90) {
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
//      char part1[5], part2[5];
//      dtostrf(kp, 4, 1, part1);
//      dtostrf(ki, 4, 1, part2);
      strcpy(line, "Kp=     Ki=     ");
//      strcat(line, part1);
//      strcat(line, " Ki=");
//      strcat(line, part2);
//      sprintf(line, "Kp=%s Ki=%s ", part1, part2);

      format_number(kp/10, 1, line+7);
      format_number(ki/10, 1, line+15);

      if(ui.state == fsm_kp) {
        line[2] = 0x7E; // right arrow
        line[7] = 0x7F; // left arrow
      } else if(ui.state == fsm_ki) {
        line[10] = 0x7E; // right arrow
        line[15] = 0x7F; // left arrow
      }
      
      Serial.println(line);
      lcd.print(line);

      // line 2 
      lcd.setCursor(0,1);
//      dtostrf(kd, 4, 1, part1);
      strcpy(line, "Kd=     Db=   S ");
//      strcat(line, part1);
//      dtostrf((float)db, 2, 0, part2);
//      strcat(line, " Db=");
//      strcat(line, part2);
//      strcat(line, (ui.state == fsm_reset) ? " R " : " S ");
//      sprintf(line, "Kd=%s Db=%2d %s ", part1, db, (ui.state == fsm_reset) ? "R" : "S");

      format_number(kd/10, 1, line+7);
      format_number(db, 0, line+13);
  
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

/*      
      // show which field is beging edited
      byte left_marker, right_marker, marker_line;
      if(ui.state == fsm_kp) {
        left_marker = 2;
        right_marker = 7;
        marker_line = 0;
      } else if(ui.state == fsm_ki) {
        left_marker = 10;
        right_marker = 15;
        marker_line = 0;
      } else if(ui.state == fsm_kd) {
        left_marker = 2;
        right_marker = 7;
        marker_line = 1;
      } else if(ui.state == fsm_db) {
        left_marker = 10;
        right_marker = 13;
        marker_line = 1;
      } else {
        left_marker = 13;
        right_marker = 15;
        marker_line = 1;
      }
      lcd.setCursor(left_marker, marker_line);      
      lcd.write(0x7E); // right arrow
      lcd.setCursor(right_marker, marker_line);      
      lcd.write(0x7F); // left arrow
*/

    }
  }
}

char *format_number(unsigned x, char shift, char *s)
{
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

void decoder()
//very short interrupt routine 
//Remember that the routine is only called when pin1
//changes state, so it's the value of pin2 that we're
//interrested in here
{
  if(millis() - rotaryTime > 5) {
    if (digitalRead(PIN_ROT_A) == digitalRead(PIN_ROT_B)) {
      trigger(E_ROT_INC); //if encoder channels are the same, direction is CW
    } else {
      trigger(E_ROT_DEC); //if they are not the same, direction is CCW
    }
    rotaryTime = millis();
  }
}

void checkButton() {
 // ignore anything that happens within 5 ms of last activity (de-bounce)
  if(buttonTime == 0) {
    boolean buttonPushed = (analogRead(6) > 512);
    if(!buttonPushed) {  // button finally release after LongClick
      isPressed = false;
      buttonTime = millis();
    }
    return;
  }
  if(millis() - buttonTime > 5) {
    if(!isPressed) {  // currently not pressed
      boolean buttonPushed = (analogRead(6) > 512);
      if(buttonPushed) {  // button now pressed
        isPressed = true;
        buttonTime = millis();
      }
    } else {  // currently pressed
      boolean buttonPushed = (analogRead(6) > 512);
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
  Serial.print("Event: ");
  Serial.println(event);
  (ui.state)(&ui, event);
  eventRaised = true;
}

void fsm_manual(StateMachine *fsm, Event event) {
  switch(event) {
    case E_ROT_INC:
      target_yaw++;
      break;
    case E_ROT_DEC:
      target_yaw--;
      break;
    case E_CLICK:
      // no-op
      break;
    case E_LONG_CLICK:
      fsm->state = fsm_kp;
      break;
  }
}

void fsm_kp(StateMachine *fsm, Event event) {
  switch(event) {
    case E_ROT_INC:
      kp+=10;
      break;
    case E_ROT_DEC:
      if(kp > 0)
        kp-=10;
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
      ki+=10;
      break;
    case E_ROT_DEC:
      if(ki > 0)
        ki-=10;
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
      kd+=10;
      break;
    case E_ROT_DEC:
      if(kd > 0)
        kd-=10;
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
      db++;
      break;
    case E_ROT_DEC:
      db--;
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
      fsm->state = fsm_manual;
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
      // restore values from EEPROM
      fsm->state = fsm_manual;
      break;
    case E_LONG_CLICK:
      fsm->state = fsm_kp;
      break;
  }
}



