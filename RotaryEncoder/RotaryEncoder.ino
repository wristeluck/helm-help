/* Rotary encoder read example */
#define ENC_A 8
#define ENC_B 9
#define ENC_PORT PINB
 
#define PIN_LCD_RS       4 
#define PIN_LCD_EN       7
#define PIN_LCD_D4      14
#define PIN_LCD_D5      15
#define PIN_LCD_D6      16
#define PIN_LCD_D7      17

#define LCD_CHARS   16
#define LCD_LINES    2
// LCD
#include <LiquidCrystal.h>
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

void setup()
{
  /* Setup encoder pins as inputs */
  pinMode(ENC_A, INPUT);
  digitalWrite(ENC_A, HIGH);
  pinMode(ENC_B, INPUT);
  digitalWrite(ENC_B, HIGH);
  Serial.begin (9600);
  lcd.print("Start");
}
 
void loop()
{
 static uint8_t counter = 0;      //this variable will be changed by encoder input
 int8_t tmpdata;
 /**/
  tmpdata = read_encoder();
  if( tmpdata ) {
    counter += tmpdata;
    lcd.clear();
    lcd.print("counter: ");
    lcd.print(counter, DEC);
  }
}
 
/* returns change in encoder state (-1,0,1) */
int8_t read_encoder()
{
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2;                   //remember previous state
  old_AB |= ( ENC_PORT & 0x03 );  //add current state
  return ( enc_states[( old_AB & 0x0f )]);
}
