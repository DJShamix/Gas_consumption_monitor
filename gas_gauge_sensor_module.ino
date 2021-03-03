#include <SPI.h>
#define _LCD_TYPE 1
#include <LCD_1602_RUS_ALL.h>

#define SPI_SS_PIN            8
#define encoder_SW_pin        4
#define encoder_DT_pin        2
#define encoder_CLK_pin       3
#include "GyverEncoder.h"
Encoder enc1(encoder_CLK_pin, encoder_DT_pin, encoder_SW_pin);

#define pressure_sens_1_pin   A6          // pressure sensor voltage input pin
#define pressure_sens_2_pin   A7
#define temperature_sens_pin  A3          // temperature sensor voltage input pin

#define pressure_sensor_shunt_res 239.4   // Ohms 
#define temp_sensor_current       0.001   // Temp sensor current is 1 mA

#define update_interval_sens       200        // Update data each 200 millseconds
#define update_interval_disp       600        // Update data each 600 millseconds

LCD_1602_RUS lcd(0x27, 16, 2);


//system_error_states
enum{
  SYS_ERROR_CLEAR = 0,
  SYS_ERROR_P1_SHORT_CIRCUIT,
  SYS_ERROR_P2_SHORT_CIRCUIT,
  SYS_ERROR_P1_OPEN_CIRCUIT,
  SYS_ERROR_P2_OPEN_CIRCUIT,
  SYS_ERROR_T_OPEN_CIRCUIT,
  SYS_ERROR_T_SHORT_CIRCUIT,
}system_error_states;

//sensors_id
enum{
  SENS_PRESSURE_1 = 1,
  SENS_PRESSURE_2,
  SENS_TEMPERATURE
}sensors_id;

/* Stores actual system params */
struct{
  int adc_temp;       // ADC values (remove in future)
  int adc_press_1;
  int adc_press_2;
  
  float temperature;
  long pressure_1;   // ПД100И-ДВ 0,01-871-0,25 // Вход 0...-10 кПа,  Выход 4...20 мА // 12...36 В Питание
  long pressure_2;   // ПД100И-ДВ 0,1-871-0,5   // Вход 0...-100 кПа, Выход 4...20 мА // 12...36 В Питание

  int pressure_error;
  int temperature_error;
}actual_state;
/*------------------------------*/


//***** encoder_actions ********* 
enum{
  ENC_NO_ACTION,
  ENC_LEFT,
  ENC_RIGHT,
  ENC_HOLD_LEFT,
  ENC_HOLD_RIGHT,
  ENC_LONG_PRESS,
  ENC_PRESS,
  ENC_CLICK,
}encoder_actions;
int list_position = 0;
String text_lines_settings[4] = { "1.Назад", "2.Вент. клапанов", "3.Настр. времени", "4.Информ. системы"};

enum{
  POS_BACK,
  POS_VALVE_BLOWOUT,
  POS_TIME_SETUP,
  POS_SYS_INFO
}display_positions;
//-------------------------------

//display_states
enum{
  STATE_MAIN_SCREEN = 0,
  STATE_SETTINGS,
  STATE_SET_BLOWOUT,
  STATE_SET_TIME,
  STATE_SYS_PARAMS
}display_states;
uint8_t display_state = STATE_SET_BLOWOUT;
uint8_t display_need_update = 0;

uint8_t valve_air_states[9] = {0};

/* Function prototypes */
void read_sensors();
float convert_ADC_volt(int adc_voltage);
void calculate_pressure(int sensor_id, int adc_voltage);
float calculate_temperature(int adc_voltage);
void update_encoder();
void do_action_with_encoder(uint8_t action);
void update_display();
void display_main_screen();
void display_settings_screen();
void display_blowout_screen();
//-------------------------------


void setup() {
  pinMode(pressure_sens_1_pin, INPUT);    // Set pins as analog input
  pinMode(pressure_sens_2_pin, INPUT);
  pinMode(temperature_sens_pin, INPUT);

  enc1.setType(TYPE2);
  enc1.setTickMode(AUTO);

  actual_state.pressure_error    = SYS_ERROR_CLEAR; // Clear errors
  actual_state.temperature_error = SYS_ERROR_CLEAR;

  lcd.init();                   //Инициализация LCD (по умолчанию для ESP8266: 4 - SDA, 5 - SCL)
  lcd.backlight();
  lcd.setCursor(0, 0);  lcd.print("Счетчик расхода");
  lcd.setCursor(5, 1);  lcd.print("газа");
  delay(2000);
  lcd.clear();

  SPI.begin();                  // Инициализация SPI для драйвера клапанов
  pinMode(SPI_SS_PIN, OUTPUT);
  SPI.transfer(0); 
  digitalWrite(SPI_SS_PIN, LOW);
  delay(10);
  digitalWrite(SPI_SS_PIN, HIGH);
    
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  static long last_update_sens = millis();
  static long last_update_disp = millis();

  //##### Encoder data update #####
  update_encoder();
  /*###############################*/


  // Sensors data update
  if(millis() >= last_update_sens + update_interval_sens){
    read_sensors();                                                 // read new data from sensors

    calculate_pressure(SENS_PRESSURE_1, actual_state.adc_press_1);  // calculate ADC to C values
    calculate_pressure(SENS_PRESSURE_2, actual_state.adc_press_2);
  
    calculate_temperature(actual_state.adc_temp);

    last_update_sens = millis();
  }
  /*###############################*/


  // Screen update
  if(millis() >= last_update_disp + update_interval_disp){

    // Do update stuff with LCD display
    update_display();

    last_update_disp = millis();
  }

  delay(2);
}
/*------------------------------*/


/* Read sensors voltage to ADC */
void read_sensors(){
  actual_state.adc_press_1 = analogRead(pressure_sens_1_pin);
  actual_state.adc_press_2 = analogRead(pressure_sens_2_pin);
  actual_state.adc_temp    = analogRead(temperature_sens_pin);
}
/*------------------------------*/


/* Converts ADC data to voltage */
float convert_ADC_volt(int adc_voltage){
  float return_value =  (5.0 / 1023.0) * adc_voltage;
  return  return_value;
}
/*------------------------------*/


/*######################################*/

void calculate_pressure_new(int sensor_id){

//  Serial.println("#######");
  
 // float sensor_voltage = (5.0 / 1023.0) * actual_state.adc_press_1;
  float sensor_voltage = convert_ADC_volt(actual_state.adc_press_1);
  Serial.print(sensor_voltage);
  Serial.print("  ");
  float sensor_current = (sensor_voltage / pressure_sensor_shunt_res)*1000000 +58.0;
  Serial.print(sensor_current);
  Serial.print("  ");

  sensor_current = 0;
  for(int i = 0; i < 10; i++){
    sensor_voltage = convert_ADC_volt(analogRead(pressure_sens_2_pin));
    sensor_current += (sensor_voltage / pressure_sensor_shunt_res);
    delay(10);
  }
  sensor_current = sensor_current *100000 +35.0;
  Serial.print(sensor_voltage);
  Serial.print("  ");
  Serial.print(sensor_current);


//  if(sensor_id == SENS_PRESSURE_1)
    actual_state.pressure_1 = map((long)(sensor_current), 4000, 20000, -10000, 0); // Pressure in kPa
    Serial.print(" | ");
    Serial.println(actual_state.pressure_1);
//  else 
//    actual_state.pressure_2 = map(sensor_current, 0.004, 0.020, -100000, 0);
//
//
//  Serial.println(actual_state.pressure_1);
//  Serial.println(actual_state.pressure_2);
//  
//  Serial.println("#######");
}

/*######################################*/


/* Calculate pressure out of the ADC voltage */
void calculate_pressure(int sensor_id, int adc_voltage){

  float sensor_voltage = convert_ADC_volt(adc_voltage);
  float sensor_current = (sensor_voltage / pressure_sensor_shunt_res) * 1000000;

  //realize conversion "current -> pressure"

  if(sensor_current < 4000){                    // Under current error
    if(sensor_id == SENS_PRESSURE_1)            // Sensor disconnected
      actual_state.pressure_error = SYS_ERROR_P1_OPEN_CIRCUIT;   
    else
    if(sensor_id == SENS_PRESSURE_2)
      actual_state.pressure_error = SYS_ERROR_P2_OPEN_CIRCUIT;
    return 0; 
  }
  else if(sensor_current > 20000){              // Over current error
    if(sensor_id == SENS_PRESSURE_1)            // Sensor shorted
      actual_state.pressure_error = SYS_ERROR_P1_SHORT_CIRCUIT;  
    else
    if(sensor_id == SENS_PRESSURE_2)
      actual_state.pressure_error = SYS_ERROR_P2_SHORT_CIRCUIT;  
    return 0; 
  }
  
  if(sensor_id == SENS_PRESSURE_1)
    actual_state.pressure_1 = round(map((long)(sensor_current), 4000, 20000, -10000, 0)); // Pressure in kPa
  else 
    actual_state.pressure_2 = round(map((long)(sensor_current), 4000, 20000, -100000, 0));

}
/*------------------------------*/


/* Calculate temperature out of the ADC voltage */
float calculate_temperature(int adc_voltage){

  float sensor_voltage = convert_ADC_volt(adc_voltage);
  float temp_resist = sensor_voltage / temp_sensor_current;

  //realize conversion "voltage -> temperature"
  actual_state.temperature = ((temp_resist / 100.0) - 1.0) / 0.00385;
}
/*------------------------------*/


/* Air valves control */
void control_valves(){

  uint8_t bit_mask = 0;

  Serial.println("#### Vents states #####");
  for(int i = 0; i < 8; i++){
    Serial.print(valve_air_states[i]);
    Serial.println("");
    
    bit_mask |= (1 && valve_air_states[i]) << i;
  }

  Serial.println("#### !!!!!!! #####");
  Serial.println("#### VENTING #####");
  Serial.println("#### !!!!!!! #####");
  Serial.println(bit_mask);

  //bit_mask = 255;

  SPI.transfer(bit_mask);
  digitalWrite(SPI_SS_PIN, LOW);
  delay(10);
  digitalWrite(SPI_SS_PIN, HIGH);  
}
/*------------------------------*/


/* Update encoder state and act */
void update_encoder(){

  if (enc1.isLeft())   do_action_with_encoder(ENC_LEFT);  
  if (enc1.isRight())  do_action_with_encoder(ENC_RIGHT);         // если был поворот

  if (enc1.isLeftH())  do_action_with_encoder(ENC_HOLD_LEFT);
  if (enc1.isRightH()) do_action_with_encoder(ENC_HOLD_RIGHT);    // если было удержание + поворот

  if (enc1.isPress())  do_action_with_encoder(ENC_PRESS);         // нажатие на кнопку (+ дебаунс)
  if (enc1.isClick())  do_action_with_encoder(ENC_CLICK);         // отпускание кнопки (+ дебаунс)

  if (enc1.isHolded()) do_action_with_encoder(ENC_LONG_PRESS);    // если была удержана и энк не поворачивался
  if (enc1.isHold())   do_action_with_encoder(ENC_NO_ACTION);     // возвращает состояние кнопки
  
}
/*------------------------------*/



void do_action_with_encoder(uint8_t action){

  static uint8_t last_display_state = display_state;
/*
    ENC_NO_ACTION,
  ENC_LEFT,
  ENC_RIGHT,
  ENC_HOLD_LEFT,
  ENC_HOLD_RIGHT,
  ENC_PRESS,
  ENC_CLICK,
  */
  
  switch(display_state){
      case STATE_MAIN_SCREEN:
         if(action == ENC_CLICK)
            display_state = STATE_SETTINGS;
            display_need_update = 1;
            list_position = 0;
        break;
  
      case STATE_SETTINGS:
        switch(action){
          case ENC_LEFT:
            if(list_position - 1 >= 0){ list_position--; display_need_update = 1; }
            break;
            
          case ENC_RIGHT:
            if(list_position + 1 <= 3){ list_position++; display_need_update = 1; }
            break;

          case ENC_CLICK:
            if(list_position == POS_BACK){
              display_state = STATE_MAIN_SCREEN;
              list_position = 0;
            }else
            
            if(list_position == POS_VALVE_BLOWOUT){
              display_state = STATE_SET_BLOWOUT;
              list_position = 0;
            }else
            
            if(list_position == POS_TIME_SETUP){
              display_state = STATE_SET_TIME;
              list_position = 0;
            }else
            
            if(list_position == POS_SYS_INFO){
              display_state = STATE_SYS_PARAMS;
              list_position = 0;
            }
            break;
        }
        break;
  
      case STATE_SET_BLOWOUT:
        switch(action){
          case ENC_LEFT:
            if(list_position - 1 >= 0){ list_position--; display_need_update = 1; }
            break;
            
          case ENC_RIGHT:
            if(list_position + 1 <= 8){ list_position++; display_need_update = 1; }
            break;

          case ENC_CLICK:
            if(list_position == 8){                           // blow all valves
              for(int i = 0; i < 8; i++) valve_air_states[i] = 1;
              control_valves(); 
              delay(1000);
              for(int i = 0; i < 8; i++) valve_air_states[i] = 0; 
              control_valves();
            } 
            else{ 
              Serial.print("List position: ");
              Serial.println(list_position);
              valve_air_states[list_position] = 1;         // blow each valve separately
              display_need_update = 1; 
              control_valves();  
              delay(1000); 
              valve_air_states[list_position] = 0;
              control_valves();
            }
            break;

          case ENC_LONG_PRESS:
            display_state = STATE_SETTINGS;
            display_need_update = 1;
            list_position = 0;
            break;
        }
        break;
  
      case STATE_SYS_PARAMS:
        if(action == ENC_CLICK || action == ENC_LONG_PRESS){
          display_state = STATE_SETTINGS;
          display_need_update = 1;
          list_position = 0;
        }
        break;  
    }

    if(last_display_state != display_state){
      last_display_state = display_state;
      lcd.clear();
    }
}
/*------------------------------*/


/* Update LCD display information */
void update_display(){

  switch(display_state){
    case STATE_MAIN_SCREEN:
      display_main_screen();
      break;

    case STATE_SETTINGS:
      display_settings_screen();
      break;

    case STATE_SET_BLOWOUT:
      display_blowout_screen();    
      break;

    case STATE_SYS_PARAMS:
      display_system_params();
      break;  
  }
}
/*------------------------------*/

/* Display main screen */
void display_main_screen(){

  Serial.print(actual_state.adc_press_1);
  Serial.print("  ");
  Serial.print(actual_state.pressure_1);
  Serial.print("  ");
  Serial.print(actual_state.adc_press_2);
  Serial.print("  ");
  Serial.println(actual_state.pressure_2);
  
  lcd.setCursor(0, 0);
  lcd.print("       ");
  lcd.setCursor(0, 0);
  lcd.print(actual_state.pressure_1, DEC);
  
  lcd.setCursor(8, 0);
  lcd.print("       ");
  lcd.setCursor(8, 0);
  lcd.print(actual_state.pressure_2, DEC);

  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(actual_state.temperature, DEC);
}
/*------------------------------*/


/* Display settings screen */
void display_settings_screen(){
  
//String text_lines_settings[4] = { "1.Назад", "2.Вент. клапанов", "3.Настр. времени", "4.Информ. системы"};
  if(display_need_update != 0){
    for(int i = 0; i < 2; i++){
      lcd.setCursor(0, 0);
      lcd.print("                ");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      
      lcd.setCursor(0, 0);  lcd.print(text_lines_settings[list_position]);
      lcd.setCursor(0, 0);  lcd.print("> ");
      if(list_position + 1 < 4){    // sizeof(text_lines_settings)-1
        lcd.setCursor(0, 1);
        lcd.print(text_lines_settings[list_position+1]);
      }else{
        lcd.setCursor(0, 1);
        lcd.print("                ");
      }
      display_need_update = 0;
    }
  }
}
/*------------------------------*/

/* Display valves blow out screen */
void display_blowout_screen(){
  
  lcd.setCursor(0, 0);
  lcd.print("Air valves vent");

  lcd.setCursor(0, 1);
  lcd.print("Valve: ");

  lcd.setCursor(8, 1);
  if(list_position == 8)
    lcd.print("all");
  else{
    lcd.print((int)(list_position+1), HEX);
    lcd.setCursor(9, 1); lcd.print("  ");
  }
}
/*------------------------------*/

/* Display system params */
void  display_system_params(){
  lcd.setCursor(0, 0);
  lcd.print("System params screen");
  lcd.setCursor(5, 1);
  lcd.print("ver/1.3");
}
/*------------------------------*/
