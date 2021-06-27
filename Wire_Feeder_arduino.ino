#include <EEPROM.h>


const int ARRAY_SIZE = 6;
const int STARTING_EEPROM_ADDRESS = 1;
int newNumbers[ARRAY_SIZE];
int  eeprom_read = 0;

#include "Nextion.h"

/*
   Declare a button object [page id:0,component id:1, component name: "b0"].
*/
NexButton bs12t = NexButton(1, 1, "bs12t");
NexButton bs14t = NexButton(1, 2, "bs14t");
NexButton bs1off = NexButton(1, 3, "bs1off");
NexButton bs1minus = NexButton(1, 4, "bs1minus");
NexButton bs1plus = NexButton(1, 5, "bs1plus");
NexText ts1percent = NexText(1, 6, "ts1percent");
//ts1percent.setText ("Engrave");

NexButton bs2minus_slow = NexButton(2, 6, "bs1minus_slow");
NexButton bs2plus_slow = NexButton(2, 9, "bs1plus_slow");
NexButton bs2minus_fast = NexButton(2, 7, "bs1minus_slow");
NexButton bs2plus_fast = NexButton(2, 8, "bs1plus_fast");
NexButton s2_wire_for = NexButton(2, 19, "s2_wire_for");
NexRadio r0 = NexRadio(2, 14, "r0");  // Radio checkbox added
NexRadio r1 = NexRadio(2, 15, "r1");  // Radio checkbox added
NexRadio r2 = NexRadio(2, 16, "r2");  // Radio checkbox added
NexRadio r3 = NexRadio(2, 17, "r3");  // Radio checkbox added
NexText s2start_var = NexText(2, 10, "s2start_var");
NexText s2for_var = NexText(2, 11, "s2for_var");
NexText s2stop_var = NexText(2, 12, "s2stop_var");
NexText s2back_var = NexText(2, 13, "s2back_var");

NexButton s2_save = NexButton(2, 18, "s2_save");

NexPage page1 = NexPage(1, 0, "page1");  // Page added as a touch event
NexPage page2 = NexPage(2, 0, "page2");  // Page added as a touch event

int init_state = 1;

char buffer[100] = {0};


//eeprom
int8_t   varA;
int8_t   varB;
int8_t   varC;
int8_t   varD;
int8_t   varE;
int8_t   varF;

uint8_t  var01 EEMEM; // 8 bit
uint8_t  var02 EEMEM; // 8 bit
uint8_t  var03 EEMEM; // 8 bit
uint8_t  var04 EEMEM; // 8 bit
uint8_t  var05 EEMEM; // 8 bit
uint8_t  var06 EEMEM; // 8 bit

/*
   Register a button object to the touch event list.
*/
NexTouch *nex_listen_list[] =
{
  &bs12t,
  &bs14t,
  &bs1off,
  &bs1minus,
  &bs1plus,
  &bs2minus_slow,
  &bs2plus_slow,
  &bs2minus_fast,
  &bs2plus_fast,
  &page1,  // Page added as a touch event
  &page2,  // Page added as a touch event
  &r0,
  &r1,
  &r2,
  &r3,
  &s2_save,
  &s2_wire_for,
  NULL
};


#define enA 3
#define in1 4
#define in2 5
#define start_button 10

int rotDirection = 0;
int pressed;

int pwmOutput;

int sprint = false;   //active Serial for Debug

int speed_value = 10;

String speed_value_string;

int setup_wire = 0;

int wire_forward = 1;
int wire_backward = 0;
int wire_forward_state = 0;
int wire_backward_state = 0;
int wire_forward_start = 0;
int wire_backward_start = 0;
int wire_stop_start = 0;
int wire_stop = 0;

int s2_wire_for_start = 0;

int wire_start_state = 0;

int welding_state = 0;              //0 Off; 1 2T, 2 4T
int welding_4t = 0;

int choose_var_input;
int change_var_input;
//int button1_state;
//int button2_state;
//int button3_state;

int lastButtonState;    // the previous state of button
int currentButtonState; // the current state of button


unsigned long previousMillis_start = 0;
long interval_start = 700;
unsigned long currentMillis_start;
int wire_intro_start = 1;

unsigned long previousMillis_forward = 0;
long interval_forward = 150;
unsigned long currentMillis_forward;
int timer_reset_forward = 1;

unsigned long previousMillis_stop = 0;
long interval_stop = 600;
unsigned long currentMillis_stop;
int timer_reset_stop = 0;

unsigned long previousMillis_backward = 0;
long interval_backward = 210;
unsigned long currentMillis_backward;
int timer_reset_backward = 0;




void bs12tPopCallback(void *ptr)
{
  welding_state = 1;
  bs12t.Set_background_color_bco(2016);
  bs14t.Set_background_color_bco(63488);
  bs1off.Set_background_color_bco(63488);
}

void bs14tPopCallback(void *ptr)
{
  welding_state = 2;
  bs12t.Set_background_color_bco(63488);
  bs14t.Set_background_color_bco(2016);
  bs1off.Set_background_color_bco(63488);
  pressed = LOW;
}

void bs1offPopCallback(void *ptr)
{
  welding_state = 0;
  bs12t.Set_background_color_bco(63488);
  bs14t.Set_background_color_bco(63488);
  bs1off.Set_background_color_bco(2016);
}

void bs1minusPopCallback(void *ptr)
{
  speed_value = speed_value - 5;

  if (speed_value >= 100) {
    speed_value = 100;
  }

  Serial.print("ts1percent.txt=\"");
  Serial.print(speed_value);
  Serial.print("%");
  Serial.write('"');
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}

void bs1plusPopCallback(void *ptr)
{
  speed_value = speed_value + 5;

  if (speed_value <= 1) {
    speed_value = 1;
  }
  Serial.print("ts1percent.txt=\"");
  Serial.print(speed_value);
  Serial.print("%");
  Serial.write('"');
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);

  //speed_value_string = String(speed_value );
  //speed_value_string = speed_value_string + "%";
  //ts1percent.setText(speed_value_string);
}

void bs2minus_slowPopCallback(void *ptr)
{
  set_var();
  change_var_input = change_var_input - 10;
  change_var();
}

void bs2plus_slowPopCallback(void *ptr)
{
  set_var();
  change_var_input = change_var_input + 10;
  change_var();
}

void bs2minus_fastPopCallback(void *ptr)
{
  set_var();
  change_var_input = change_var_input - 100;
  change_var();
}

void bs2plus_fastPopCallback(void *ptr)
{
  set_var();
  change_var_input = change_var_input + 100;
  change_var();
}


void s2_savePopCallback(void *ptr)
{
  saveEprom();
  s2_save.Set_background_color_bco(2016);
  s2_save.setText("Saved");
}

void s2_wire_forPopCallback(void *ptr)
{
  setup_wire = !setup_wire;
  if (setup_wire == 0) {
    s2_wire_for.Set_background_color_bco(50712);
    s2_wire_for_start = 0;
    wire_start_state = 0;
  }
  if (setup_wire == 1) {
    s2_wire_for.Set_background_color_bco(2016);
    s2_wire_for_start = 1;
  }
}


void change_var() {
  if (change_var_input <= 0) {
    change_var_input = 0;
  }

  if (choose_var_input == 0) {
    interval_start = change_var_input;
    send_txt();
  }
  if (choose_var_input == 1) {
    interval_forward = change_var_input;
    send_txt();
  }
  if (choose_var_input == 2) {
    interval_stop = change_var_input;
    send_txt();
  }
  if (choose_var_input == 3) {
    interval_backward = change_var_input;
    send_txt();
  }

  //choose_var_input;
}

void set_var() {
  if (choose_var_input == 0) {
    change_var_input = interval_start;
  }
  if (choose_var_input == 1) {
    change_var_input = interval_forward;
  }
  if (choose_var_input == 2) {
    change_var_input = interval_stop;
  }
  if (choose_var_input == 3) {
    change_var_input = interval_backward;
  }
}

// Page change event:
void page1PushCallback(void *ptr)  // If page 0 is loaded on the display, the following is going to execute:
{
  if (welding_state == 0) {
    bs12t.Set_background_color_bco(63488);
    bs14t.Set_background_color_bco(63488);
    bs1off.Set_background_color_bco(2016);
  }
  if (welding_state == 1) {
    bs12t.Set_background_color_bco(2016);
    bs14t.Set_background_color_bco(63488);
    bs1off.Set_background_color_bco(63488);
  }
  if (welding_state == 2) {
    bs12t.Set_background_color_bco(63488);
    bs14t.Set_background_color_bco(2016);
    bs1off.Set_background_color_bco(63488);
  }
  Serial.print("ts1percent.txt=\"");
  Serial.print(speed_value);
  Serial.print("%");
  Serial.write('"');
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
  saveEprom();
}

// Page change event:
void page2PushCallback(void *ptr)  // If page 0 is loaded on the display, the following is going to execute:
{
  send_txt();
  if (setup_wire == 0) {
    s2_wire_for.Set_background_color_bco(50712);
  }
  if (setup_wire == 1) {
    s2_wire_for.Set_background_color_bco(2016);
  }
    s2_save.Set_background_color_bco(50712);
  s2_save.setText("Save");
}

void send_txt() {

  //s2_wire_for.Set_background_color_bco(2016);

  Serial.print("s2start_var.txt=\"");
  Serial.print(interval_start);
  Serial.print(" ms");
  Serial.write('"');
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);

  Serial.print("s2for_var.txt=\"");
  Serial.print(interval_forward);
  Serial.print(" ms");
  Serial.write('"');
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);

  Serial.print("s2stop_var.txt=\"");
  Serial.print(interval_stop);
  Serial.print(" ms");
  Serial.write('"');
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);

  Serial.print("s2back_var.txt=\"");
  Serial.print(interval_backward);
  Serial.print(" ms");
  Serial.write('"');
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}

void r0PushCallback(void *ptr)  // Press event for radio checkbox
{
  choose_var_input = 0;
}

void r1PushCallback(void *ptr)  // Press event for radio checkbox
{
  choose_var_input = 1;
}
void r2PushCallback(void *ptr)  // Press event for radio checkbox
{
  choose_var_input = 2;
}
void r3PushCallback(void *ptr)  // Press event for radio checkbox
{
  choose_var_input = 3;
}
void setup() {

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(start_button, INPUT);
  // Set initial rotation direction
  //digitalWrite(in1, LOW);
  //digitalWrite(in2, HIGH);
  currentButtonState = digitalRead(start_button);
  /* Set the baudrate which is for debug and communicate with Nextion screen. */
  nexInit();

  /* Register the pop event callback function of the current button component. */
  bs12t.attachPop(bs12tPopCallback, &bs12t);
  bs14t.attachPop(bs14tPopCallback, &bs14t);
  bs1off.attachPop(bs1offPopCallback, &bs1off);
  bs1minus.attachPop(bs1minusPopCallback, &bs1minus);
  bs1plus.attachPop(bs1plusPopCallback, &bs1plus);
  bs2minus_slow.attachPop(bs2minus_slowPopCallback, &bs2minus_slow);
  bs2plus_slow.attachPop(bs2plus_slowPopCallback, &bs2plus_slow);
  bs2minus_fast.attachPop(bs2minus_fastPopCallback, &bs2minus_fast);
  bs2plus_fast.attachPop(bs2plus_fastPopCallback, &bs2plus_fast);
  s2_wire_for.attachPop(s2_wire_forPopCallback, &s2_wire_for);
  page1.attachPush(page1PushCallback);  // Page press event
  page2.attachPush(page2PushCallback);  // Page press event
  r0.attachPush(r0PushCallback);
  r1.attachPush(r1PushCallback);
  r2.attachPush(r2PushCallback);
  r3.attachPush(r3PushCallback);
  s2_save.attachPop(s2_savePopCallback, &s2_save);


}

void loop() {

  nexLoop(nex_listen_list);
  wire_detect();
  wire_start();
  if (init_state == 1) {
    delay(3000);
    page1.show();
    init_state = 0;
    bs12t.Set_background_color_bco(63488);
    bs14t.Set_background_color_bco(63488);
    bs1off.Set_background_color_bco(2016);
    read_eeprom();
    Serial.print("ts1percent.txt=\"");
    Serial.print(speed_value);
    Serial.print("%");
    Serial.write('"');
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFF);

  }


}

void wire_detect() {

  pwmOutput = map(speed_value, 0, 100, 55 , 255); // Map the value from 0 to 100


  currentMillis_start = millis();
  if ( welding_state == 1) {   //2T feeder
    if (digitalRead(start_button) == true) {

      if (wire_intro_start == 0) {
        previousMillis_start = currentMillis_start;
        wire_intro_start = 1;
      }
      if (currentMillis_start - previousMillis_start >= interval_start) {

        wire_start_state = 1;
        wire_intro_start = 0;

      }
    }
    else {

      wire_start_state = 0;
      wire_intro_start = 0;
    }
  }

  if ( welding_state == 2) {   //4T feeder

    if (digitalRead(start_button) == true) {
      // Serial.println("The button is pressed");
      delay(500);
      if (pressed == LOW) {
        pressed = HIGH;
      } else {
        pressed = LOW;
      }

      if (wire_intro_start == 0) {
        previousMillis_start = currentMillis_start;
        wire_intro_start = 1;
      }
    }
    if (pressed == HIGH) {
      if (currentMillis_start - previousMillis_start >= interval_start) {
        wire_start_state = 1;
        wire_intro_start = 0;
      }
    } else {
      // Serial.println("Wire feed off");
      wire_start_state = 0;
      wire_intro_start = 0;
    }


  }
}

void wire_start() {
  if (wire_start_state == 1) {
    if (wire_forward == 0) {
      previousMillis_forward = currentMillis_forward;
    }
    if (wire_backward == 0) {
      previousMillis_backward = currentMillis_backward;
    }
    if (wire_stop == 0) {
      previousMillis_stop = currentMillis_stop;
    }
    currentMillis_stop = millis();
    currentMillis_backward = millis();
    currentMillis_forward = millis();
    if (currentMillis_forward - previousMillis_forward >= interval_forward) {
      wire_forward = 0;                    //deactivate millis forward timer
      wire_forward_state = 1;               //set state for forward timer to change from stop timer
      previousMillis_stop = currentMillis_stop;
      previousMillis_forward = currentMillis_forward;
      wire_stop = 1;
      analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
      digitalWrite(in1, LOW);      //forward
      digitalWrite(in2, LOW);
      //  Serial.println("Wire_stop");
    }


    if (currentMillis_stop - previousMillis_stop >= interval_stop) {
      if (wire_forward_state == 1) {
        wire_backward = 1;
        previousMillis_backward = currentMillis_backward;
        wire_forward_state = 0;
        analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
        digitalWrite(in1, HIGH);      //backwards
        digitalWrite(in2, LOW);
        //    Serial.println("Wire_forward");
      } else if (wire_backward_state == 1) {
        wire_forward = 1;                             //switch to forward timer
        previousMillis_forward = currentMillis_forward;
        wire_backward_state = 0;
        analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
        digitalWrite(in1, LOW);      //backwards
        digitalWrite(in2, HIGH);
        // Serial.println("Wire_backward");
      }
      previousMillis_stop = currentMillis_stop;
      wire_stop = 0;

    }


    if (currentMillis_backward - previousMillis_backward >= interval_backward) {
      //  Serial.println("Wire_stop");
      wire_backward = 0;
      wire_backward_state = 1;
      previousMillis_stop = currentMillis_stop;
      previousMillis_backward = currentMillis_backward;
      wire_stop = 1;
      analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
      digitalWrite(in1, LOW);      //backwards
      digitalWrite(in2, LOW);
    }
  } else {
    analogWrite(0, pwmOutput); // Send PWM signal to L298N Enable pin
    digitalWrite(in1, LOW);      //backwards
    digitalWrite(in2, LOW);
    // Serial.println("Wire_stop");
    wire_forward = 1;
  }
  if (s2_wire_for_start == 1) {
    analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
    digitalWrite(in1, HIGH);      //backwards
    digitalWrite(in2, LOW);
  }
}
void saveEprom()
{

  eeprom_read = 1;
  int numbers[ARRAY_SIZE] = { eeprom_read, interval_start, interval_forward, interval_stop, interval_backward, speed_value };
  writeIntArrayIntoEEPROM(STARTING_EEPROM_ADDRESS, numbers, ARRAY_SIZE);

}

void read_eeprom() {
  delay(1000);
  readIntArrayFromEEPROM(STARTING_EEPROM_ADDRESS, newNumbers, ARRAY_SIZE);
  eeprom_read = newNumbers[0];
  if (eeprom_read == 1) {
    interval_start = newNumbers[1];
    interval_forward = newNumbers[2];
    interval_stop = newNumbers[3];
    interval_backward = newNumbers[4];
    speed_value = newNumbers[5];
   /* Serial.println("interval_start ");
    Serial.println(interval_start);*/
/*         for (int i = 0; i < ARRAY_SIZE; i++)
  {
    Serial.println(newNumbers[i]);
  }*/

  }
}

void writeIntArrayIntoEEPROM(int address, int numbers[], int arraySize)
{
  int addressIndex = address;
  for (int i = 0; i < arraySize; i++)
  {
    EEPROM.write(addressIndex, numbers[i] >> 8);
    EEPROM.write(addressIndex + 1, numbers[i] & 0xFF);
    // EEPROM.commit();
    addressIndex += 2;
  }
}

void readIntArrayFromEEPROM(int address, int numbers[], int arraySize)
{
  int addressIndex = address;
  for (int i = 0; i < arraySize; i++)
  {
    numbers[i] = (EEPROM.read(addressIndex) << 8) + EEPROM.read(addressIndex + 1);
    addressIndex += 2;
  }
}
