#include <OneWire.h>

/*
  Скетч к проекту "Универсальный пульт для шлагаумов и люср RF 433.96MHz / 315MHz с OLED дисплеем и хранением 30 ключей в памяти EEPROM"
  Аппаратная часть построена на Arduino Pro Mini 3.3v
  Исходники на GitHub: https://github.com/AlexMalov/RadioSniffer/
  Автор: МЕХАТРОН DIY, AlexMalov, 2020
  v 1.1
*/

#include <EEPROM.h>
#include <C:\Users\ander\Documents\arduino-1.8.12\hardware\tools\avr\avr\include\avr\sleep.h>
#include <C:\Users\ander\Documents\arduino-1.8.12\hardware\tools\avr\avr\include\avr\power.h>
//#include <C:\Users\ander\Documents\Arduino\433snifer\RadioSniffer-master\libs\OLED_I2C\OLED_I2C.h>
#include "pitches.h"
//#include "C:\Users\ander\Documents\Arduino\433snifer\RadioSniffer-master\libs\GyverButton\GyverButton.h"
#include <OLED_I2C.h>
#include <GyverButton.h>

//settings
#define prescal clock_div_4                     // делитель такотовой частоты
#define serial_brate (uint32_t)9600 << prescal  // задаем скорость монитора порта 9600
#define maxDataLog 160                          // длинна массива лог переключений. Максимум 64 bit + 2 sync bit 156
#define minPause 5000 >> prescal                // минимальная длинна синхроимпуьса мкс
#define maxPause 26000 >> prescal               // максимальная длинна синхроимпуьса мкс
//pins
#define rxPin 2         // выход приемника int0
#define RVR_Vcc_Pin 4   // включение питания приемника
#define txPin 9         // вход передатчика
#define TM_Vcc_Pin 8    // питание передачика
#define TM_gnd_Pin 7    // земля передатчика
#define speakerPin 10   // Спикер, он же buzzer, он же beeper
#define spr_gnd_Pin 11  // Земля спикера
#define G_Led 14        // всроенный свеодиод
#define Btn_ok_Pin 3    // Кнопка оправки текущего кода
#define Btn_left_Pin 5  // Кнопка влево
#define Btn_right_Pin 6 // Кнопка вправо


#define bounce_interval       30
#define base_move_pixels      5
#define exponential_bound     15
#define exponential_base      1.2
// pin definition
//#define btn_pin               2
#define right_pin             10
#define left_pin              11
#define down_pin              12
#define up_pin                13
//#define blu_led_pin         14  //shows operative act
#define red_led_pin         15  //error
#define grn_led_pin         16 //success
#define white_led_pin         17 //push buttton

class Direction {
public:
  Direction(int pin1, int pin2) {
    this->pins[0] = pin1;
    this->pins[1] = pin2;
    pinMode(this->pins[0], INPUT);
    pinMode(this->pins[1], INPUT);
  };
  int read_action() {
    for(int i = 0; i < 2; ++i) {
      this->current_actions[i] = digitalRead(this->pins[i]);
      this->current_action_times[i] = millis();
      if(this->current_actions[i] != this->last_actions[i]) {
        this->last_actions[i] = this->current_actions[i];
        exponential = (exponential_bound - (this->current_action_times[i] - this->last_action_times[i]));
        exponential = (exponential > 0) ? exponential : 1;
        move_multiply = exponential_base;
        for(int i = 0; i < exponential; ++i) {
          move_multiply *= exponential_base;
        }
        this->last_action_times[i] = this->current_action_times[i];
        if(i == 0) {
          return (-1) * base_move_pixels * move_multiply;
        } else {
          return base_move_pixels * move_multiply;
        }
      }
    }
    return 0;
  };
private:
  int pins[2];
  int current_actions[2];
  int last_actions[2];
  int  exponential;
  double move_multiply;
  unsigned long current_action_times[2];
  unsigned long last_action_times[2];
};

// button and debounce
//int btn_state;
//int btn_read_state;
//unsigned long btn_current_action_time;
//unsigned long btn_last_action_time;

// mouse move
int x_move, y_move;
boolean   goright, goleft, goup, godown=false;
  
Direction x_direction(left_pin, right_pin);
Direction y_direction(up_pin, down_pin);




GButton btn_ok(Btn_ok_Pin);       // кнопка ОК
GButton btn_left(Btn_left_Pin);   // кнопка влево
GButton btn_right(Btn_right_Pin); // кнопка вправо
OLED myOLED(SDA, SCL); //создаем экземпляр класса OLED с именем myOLED
extern uint8_t SmallFont[];
extern uint8_t BigNumbers[];

volatile bool recieved = false;       //что-то прочиталось
volatile int keyRawLog[maxDataLog];   // лог переключений максимум 66 bit + 1 sync bit
volatile byte logLen;                 // фактическая длинна лога
volatile bool SleepOn = false;        // режим энергосережения

enum emKeys {kUnknown, kP12bt, k12bt, k24bt, k64bt, kKeeLoq, kANmotors64};    // тип оригинального ключа
enum emSnifferMode {smdNormal, smdAutoRec, smdAutoRecSilence} snifferMode;          // режим раоы сниффера

struct tpKeyRawData{  
  byte keyID[9];            // шифр ключа 12-66 bit
  int zero[2];              // шаблон 1
  int one[2];               // шаблон 0
  int prePulse[2];          // шаблон стартовой последовательности, если есть
  int startPause[2];        // шаблон стартовой паузы
  int midlePause[2];        // шаблон средней паузы
  byte prePulseLenth;       // длинна стартовой последовательноси в битах 11
  byte codeLenth;           // длинна keyID в битах
  byte firstDataIdx;        // номер первого бита данных ключа
  emKeys type;              // тип пульта
  byte rawDataLenth;        // фактическая длинна исходника ключа
  int rawData[maxDataLog];  // исходная запись ключа
};

struct tpKeyData{  
  byte keyID[9];            // шифр ключа 12-66 bit
  int zero[2];              // шаблон 1
  int one[2];               // шаблон 0
  int prePulse[2];          // шаблон стартовой последовательности, если есть
  int startPause[2];        // шаблон стартовой паузы
  int midlePause[2];        // шаблон средней паузы
  byte prePulseLenth;       // длинна стартовой последовательноси в битах 11
  byte codeLenth;           // длинна keyID в битах
  byte firstDataIdx;        // номер первого бита данных ключа
  emKeys type;              // тип пульта
} keyData1;

byte maxKeyCount = EEPROM.length() / sizeof(tpKeyData); // максимальное кол-во ключей, которое влазит в EEPROM, но не > 40
byte EEPROM_key_count;                    // количество ключей 0..maxKeyCount, хранящихся в EEPROM
byte EEPROM_key_index = 0;                // 1..EEPROM_key_count номер последнего записанного в EEPROM ключа  
unsigned long stTimer = 0;                // тамер сна

void OLED_printKey(tpKeyData* kd, byte msgType = 0){
  String st;
  switch (snifferMode){
    case smdNormal: myOLED.clrScr(); myOLED.print("N", RIGHT, 24); break; 
    case smdAutoRec: myOLED.clrScr(); myOLED.print("A", RIGHT, 24); break; 
    case smdAutoRecSilence: return; 
  }
  switch (msgType){
    case 0: st = "The key " + String(EEPROM_key_index) + " of " + String(EEPROM_key_count) + " in ROM"; break;      
    case 1: st = "Hold the Btn to save";  break; 
    case 3: st = "The key " + String(indxKeyInROM(kd)) + " exists in ROM";  break;   
  }
  myOLED.print(st, 0, 0);  
  st = "";
  for (byte i = 0; i < kd->codeLenth >> 3; i++) st += String(kd->keyID[i], HEX) + ":";
  myOLED.print(st, 0, 12);
  st = "Type " + getTypeName(kd->type);
  myOLED.print(st, 0, 24);
  myOLED.update();
}

void OLED_printError(String st, bool err = true){
  switch (snifferMode){
    case smdNormal: myOLED.clrScr(); myOLED.print("N", RIGHT, 24); break; 
    case smdAutoRec: myOLED.clrScr(); myOLED.print("A", RIGHT, 24); break; 
    case smdAutoRecSilence: return; 
  }
  if (err) myOLED.print(F("Error!"), 0, 0);
    else myOLED.print(F("OK"), 0, 0);
  myOLED.print(st, 0, 12);  
  myOLED.update();
}

byte indxKeyInROM(tpKeyData* kd){ //возвращает индекс или ноль если нет в ROM
  bool eq = true; byte* buf = (byte*)kd;
  for (byte j = 1; j<=EEPROM_key_count; j++){  // ищем ключ в eeprom. 
    byte i = 0;
    if ((kd->type == kKeeLoq) || (kd->type == kANmotors64)) i = 4;  // для эих ключей первая часть кода переменная
    for (; i < kd->codeLenth >> 3; i++) 
      if (buf[i] != EEPROM[i+j*sizeof(tpKeyData)]) { eq = false; break;}
    if (eq) return j;
    eq = true;
  }
  return 0;
}

bool EPPROM_AddKey(tpKeyData* kd){
  byte indx;
  indx = indxKeyInROM(kd);                 // ищем ключ в eeprom. Если находим, то не делаем запись, а индекс переводим в него
  if ( indx != 0) { 
    EEPROM_key_index = indx;
    EEPROM.update(1, EEPROM_key_index);
    return false; 
  }
  if (EEPROM_key_count <= maxKeyCount) EEPROM_key_count++;
  if (EEPROM_key_count < maxKeyCount) EEPROM_key_index = EEPROM_key_count;
    else EEPROM_key_index++;
  if (EEPROM_key_index > EEPROM_key_count) EEPROM_key_index = 1;
  Serial.println(F("Adding to EEPROM"));
  for (byte i = 0; i < kd->codeLenth >> 3; i++) {
    Serial.print(kd->keyID[i], HEX); Serial.print(F(":"));  
  }
  Serial.println();
  EEPROM.put(EEPROM_key_index*sizeof(tpKeyData), *kd);
  EEPROM.update(0, EEPROM_key_count);
  EEPROM.update(1, EEPROM_key_index);
  return true;
}

void EEPROM_get_key(byte EEPROM_key_index1, tpKeyData* kd){
  int address = EEPROM_key_index1*sizeof(tpKeyData);
  if (address > EEPROM.length()) return;
  EEPROM.get(address, *kd);
}

void setup() {
  clock_prescale_set(prescal);
  btn_ok.setDebounce(50 >> prescal);
  btn_ok.setTimeout(500 >> prescal);
  btn_ok.setClickTimeout(300 >> prescal);
  btn_ok.setStepTimeout(200 >> prescal);
  btn_left.setDebounce(50 >> prescal);
  btn_left.setTimeout(500 >> prescal);
  btn_left.setClickTimeout(300 >> prescal);
  btn_left.setStepTimeout(200 >> prescal);
  btn_right.setDebounce(50 >> prescal);
  btn_right.setTimeout(500 >> prescal);
  btn_right.setClickTimeout(300 >> prescal);
  btn_right.setStepTimeout(200 >> prescal);
  btn_left.setTickMode(AUTO);
  btn_right.setTickMode(AUTO);

//  pinMode(blu_led_pin, OUTPUT);
  pinMode(red_led_pin, OUTPUT);
  pinMode(grn_led_pin, OUTPUT);
  pinMode(white_led_pin, OUTPUT);
//  pinMode(btn_pin, INPUT);
  
  pinMode(RVR_Vcc_Pin, OUTPUT); digitalWrite(RVR_Vcc_Pin, HIGH);  // включение питания приемника
  pinMode(TM_gnd_Pin, OUTPUT); digitalWrite(TM_gnd_Pin, LOW); // земля передатчика
  pinMode(TM_Vcc_Pin, OUTPUT); digitalWrite(TM_Vcc_Pin, LOW); // питание передачика отключено
  pinMode(txPin, OUTPUT); digitalWrite(txPin, LOW);           // вход передатчика
  pinMode(speakerPin, OUTPUT);                                // Спикер, он же buzzer, он же beeper
  pinMode(spr_gnd_Pin, OUTPUT);                               // Земля спикера
  myOLED.begin(SSD1306_128X32);
  Serial.begin(serial_brate);
  pinMode(G_Led, OUTPUT);
  myOLED.clrScr();                                          //Очищаем буфер дисплея.
  myOLED.setFont(SmallFont);                                //Перед выводом текста необходимо выбрать шрифт
  myOLED.print(F("Hello, read a key..."), LEFT, 0);
  const char st[16] = {98, 121, 32, 77, 69, 88, 65, 84, 80, 79, 72, 32, 68, 73, 89, 0};
  myOLED.print(st, LEFT, 24);
  myOLED.update();
  snifferMode = smdNormal;
  Sd_StartOK();
  keyData1.codeLenth = 0;
  EEPROM_key_count = EEPROM[0];
  if (EEPROM_key_count > maxKeyCount) EEPROM_key_count = 0;
  if (EEPROM_key_count != 0 ) {
    digitalWrite(G_Led, HIGH);
    EEPROM_key_index = EEPROM[1];
    Serial.print(F("Read key code from EEPROM: "));
    EEPROM_get_key(EEPROM_key_index, &keyData1);
    for (byte i = 0; i < 8; i++) { Serial.print(keyData1.keyID[i], HEX); Serial.print(F(":")); }
    Serial.println();
    delay(1000 >> prescal);
    OLED_printKey(&keyData1);
    digitalWrite(G_Led, LOW);
  } else {
    myOLED.print(F("ROM has no keys yet."), 0, 12);
    myOLED.update();  
  }
  attachInterrupt(digitalPinToInterrupt(rxPin), handleInt, CHANGE);    //pin 2 int0
  attachInterrupt(digitalPinToInterrupt(Btn_ok_Pin), wakeUp, CHANGE); //проснемся от нажаия кнопки OK
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  ACSR &= ~(1 << ACD);                  // отключаем компаратор
  DIDR0 |= 0b1111;                      // отключаем цифровые пины АЦП
  ADCSRA &= ~(1 << ADEN);               // Отключаем АЦП
  ADCSRB &= ~(1 << ACME);               // отключаем мультиплексор
  PRR |= (1 << PRADC) | (1 << PRSPI) | (1 << PRTIM1);      // Отключаем clock АЦП и шину SPI и таймер1
}

String getTypeName(emKeys tp){
  switch (tp){
    case kUnknown: return F(" Unknown");
    case kP12bt: return F(" Pre 12bit");
    case k12bt: return F(" 12bit");
    case k24bt: return F(" 24bit");
    case kKeeLoq: return F(" KeeLoq");
    case kANmotors64: return F(" ANmotors");
  }
}

void handleInt(){
  static byte changeCnt = 0;
  static byte repeatCnt = 0;
  if (recieved) {
    repeatCnt = 0;
    changeCnt = 0;
    return;
  }
  static unsigned long lastTime = 0;
  const unsigned long curTime = micros();
  const int duration = curTime - lastTime;
  lastTime = curTime;    
  
  if (((duration < minPause)||(duration > maxPause))&&(changeCnt == 0)) return;   //Ждем начальный синхроимпуль
  if ((duration > minPause)&&(duration < maxPause)&&(changeCnt >= 24)){     //похоже на очередной синхроимпульс
    const int delta = duration-abs(keyRawLog[0]);
    if (abs(delta) < (350 >> prescal)) { // это точно повторный сигнал
      repeatCnt++;
      if (repeatCnt>=2){
        recieved = true;
        logLen = changeCnt;
        repeatCnt = 0;
        changeCnt = 0;
        return;
      } 
      changeCnt = 0;
    }  
  }
  if ((duration > minPause)&&(duration < maxPause)&&(changeCnt > 0)&&(changeCnt < 20)){     //очередной синхроимпульс но мало полученных данных
    changeCnt = 0;
    repeatCnt = 0;
  }
  if ((repeatCnt>0)&&(abs(duration-abs(keyRawLog[changeCnt])) > (100 >> prescal) )){ //сравниваем с предыдущей посылкой
    changeCnt = 0;
    repeatCnt = 0;
    return;    
  }
  if (!(1&(PIND >> 2))) keyRawLog[changeCnt] = duration;            //if (!digitalRead(rxPin))
    else keyRawLog[changeCnt] = -duration;
  changeCnt++;
  if (changeCnt >= maxDataLog) { // слишком длинная посылка
    changeCnt = 0;
    repeatCnt = 0;
  }
}

bool convert2Key(tpKeyData* kd){
  long zero[2] = {0, 0}, one[2] = {0, 0};
  kd->prePulseLenth = 0;
  kd->startPause[0] = keyRawLog[logLen - 1];
  kd->startPause[1] = keyRawLog[0];         // шаблон стартовой паузы
  kd->midlePause[0] = 0;                    // шаблон средней паузы
  kd->midlePause[1] = 0;
  byte i = 1, k = 0, k0 = 0, k1 = 0, j = 0;
  unsigned int halfT = (abs(keyRawLog[i])+abs(keyRawLog[i+1])) >> 1;
  if (logLen > 131) {                              //Keeloq 64 bit коды со случайным сегментом и длинной преамулой
    for (; i < logLen; i++) {
      if (abs(keyRawLog[i]) > (2000  >> prescal)) break;  //находим второй синхроимпульс
      one[0] += keyRawLog[i];
      i++;
      if (abs(keyRawLog[i]) > (2000  >> prescal)) break;  //находим второй синхроимпульс
      one[1] += keyRawLog[i];
    }
    if (i>100) return false;
    kd->prePulseLenth = i-2;  // длинна стартовой последовательноси в битах 11
    kd->midlePause[0] = keyRawLog[i-1];      // шаблон средней паузы
    kd->midlePause[1] = keyRawLog[i];
    i++;
    kd->prePulse[0] = (one[0] << 1) / kd->prePulseLenth;        // шаблон стартовой последовательности
    kd->prePulse[1] = (one[1] << 1) / kd->prePulseLenth;
    one[0] = 0; one[1] = 0;
  }
  kd->firstDataIdx = i;
  kd->codeLenth = (logLen - i) >> 1;
  halfT = (abs(keyRawLog[i])+abs(keyRawLog[i+1])) >> 1;
  for (; i < logLen; i+=2) {
    if (abs(keyRawLog[i]) > halfT) {
      bitSet(kd->keyID[k >> 3], 7-j);
      one[0] += keyRawLog[i];
      one[1] += keyRawLog[i+1];
      k1++;
    } else {
      bitClear(kd->keyID[k >> 3], 7-j);
      zero[0] += keyRawLog[i];
      zero[1] += keyRawLog[i+1];
      k0++;
    }
    j++; if (j>7) j=0;
    k++; if (k >= kd->codeLenth) break;
  }
  kd->one[0] = one[0] / k1;
  kd->one[1] = one[1] / k1;
  kd->zero[0] = zero[0] / k0;
  kd->zero[1] = zero[1] / k0;

  switch (kd->codeLenth){
    case 12: if (kd->prePulseLenth == 0) kd->type = k12bt;
      else kd->type = kP12bt; break;
    case 24: kd->type = k24bt; break;
    case 64: kd->type = k64bt; break;
    case 65: if (kd->keyID[2] == kd->keyID[3]) kd->type = kANmotors64;
      else kd->type = kKeeLoq; break;
    default:  kd->type = kUnknown;
  }
  return true;
}

bool convert2KeyRaw(tpKeyRawData* kd){
  kd->rawDataLenth = logLen;
  for (byte i = 0; i < logLen; i++) kd->rawData[i] = keyRawLog[i];
  return convert2Key((tpKeyData*)kd);
}


void myDelayMcs(unsigned long dl){
  if (dl > 16000) delay(dl / 1000);
    else delayMicroseconds(dl);
}

void sendRawKey(tpKeyRawData* kd){
  recieved = true;
  digitalWrite(TM_Vcc_Pin, HIGH); // включаем передачик
  digitalWrite(RVR_Vcc_Pin, LOW);  // выключаем приемник
  delay(1);
  if (kd->rawData[kd->rawDataLenth-1] > 0){
    digitalWrite(txPin, HIGH);
    myDelayMcs(kd->rawData[kd->rawDataLenth-1]);
  }else{
    digitalWrite(txPin, LOW);
    myDelayMcs(-kd->rawData[kd->rawDataLenth-1]);
  }
  for (byte k = 0; k < 10; k++)
    for (byte i = 0; i < kd->rawDataLenth; i++){
      if (kd->rawData[i] > 0){
        digitalWrite(txPin, HIGH);
        myDelayMcs(kd->rawData[i]);
      }else{
        digitalWrite(txPin, LOW);
        myDelayMcs(-kd->rawData[i]);
      }
    }
  digitalWrite(txPin, LOW);
  digitalWrite(TM_Vcc_Pin, LOW); // выключаем передачик
  digitalWrite(RVR_Vcc_Pin, HIGH);  //включаем приемник
  recieved = false;  
}

void sendSynthBit(int bt[2]){
  if (bt[0] == 0) return;
  for (byte i=0; i < 2; i++){
    if (bt[i] > 0){
      PORTB |= 1 << 1; //digitalWrite(txPin, HIGH); Pin 9 arduino
      myDelayMcs(bt[i]);
    } else{
      PORTB &= ~(1 << 1); //digitalWrite(txPin, LOW);
      myDelayMcs(-bt[i]);
    }    
  }
}

void sendSynthKey(tpKeyData* kd){
  recieved = true;
  digitalWrite(TM_Vcc_Pin, HIGH); // включаем передачик
  digitalWrite(RVR_Vcc_Pin, LOW);  // выключаем приемник
  delay(4);
  randomSeed(millis());
  byte ANmotorsByte = random(256);
  for (byte k = 0; k < 4; k++){
    sendSynthBit(kd->startPause);
    if (kd->prePulseLenth > 0){
      for (byte i = 0; i < (kd->prePulseLenth)>>1; i++)
        sendSynthBit(kd->prePulse);
    }
    sendSynthBit(kd->midlePause);
    byte j = 0, bt; 
    for (byte i = 0; i < kd->codeLenth; i++){
      if ( ((i>>3) >= 2) && ((i>>3) <= 3)&&(kd->type == kANmotors64)) bt = 1&(ANmotorsByte >> (7-j)); // заменяем 2 и 3 айты на случайное число для ANmotors 
        else bt = 1&(kd->keyID[i >> 3] >> (7-j));
      if (bt) sendSynthBit(kd->one);
        else sendSynthBit(kd->zero);
      j++; if (j>7) j=0;
    }
  }
  digitalWrite(txPin, LOW);
  digitalWrite(TM_Vcc_Pin, LOW); // выключаем передачик
  digitalWrite(RVR_Vcc_Pin, HIGH);  //включаем приемник
  recieved = false;
}

void printDebugData(){
  Serial.print(F(" codeLenth ")); Serial.print(keyData1.codeLenth);
  Serial.print(F(", firstDataIdx ")); Serial.print(keyData1.firstDataIdx);
  Serial.print(F(", Key type ")); Serial.print(getTypeName(keyData1.type));
  Serial.print(F(", zero [")); Serial.print(keyData1.zero[0] << prescal); Serial.print(", "); Serial.print(keyData1.zero[1] << prescal);
  btn_ok.tick(); btn_left.tick(); btn_right.tick();
  Serial.print(F("], one [")); Serial.print(keyData1.one[0] << prescal); Serial.print(", "); Serial.print(keyData1.one[1] << prescal); Serial.print("]");
  Serial.print(F(", startPause [")); Serial.print(keyData1.startPause[0] << prescal); Serial.print(", "); Serial.print(keyData1.startPause[1] << prescal); Serial.print("]");  
  if (keyData1.prePulseLenth > 0){
    Serial.print(F(", prePulseLenth ")); Serial.print(keyData1.prePulseLenth  << prescal);
    Serial.print(F(", prePulse [")); Serial.print(keyData1.prePulse[0] << prescal); Serial.print(", "); Serial.print(keyData1.prePulse[1] << prescal); Serial.print("]");
  }
  if (abs(keyData1.midlePause[0]) > 0){
    Serial.print(F(", Header [")); Serial.print(keyData1.midlePause[0] << prescal); Serial.print(", "); Serial.print(keyData1.midlePause[1] << prescal); Serial.print("]"); 
  }
  Serial.println();
  for (byte i = 0; i < logLen; i++) {
    Serial.print(keyRawLog[i] << prescal); Serial.print(", ");
    btn_ok.tick(); btn_left.tick(); btn_right.tick();
  }
  Serial.print(F(" rawLen ")); Serial.println(logLen);
}

void wakeUp(){
  btn_ok.tick();
}

void go2sleep(){
  if (SleepOn) return;
  SleepOn = true;
  digitalWrite(RVR_Vcc_Pin, LOW);  //включаем приемник
  Serial.print(F("sleeping..."));
  myOLED.sleepMode(SLEEP_ON);
  delay(5);
  cli();
  sleep_enable();
  sleep_bod_disable();  // Отключаем детектор пониженного напряжения питания
  sei();
  sleep_cpu();
  sleep_disable();
  //просыпаемся от прерывания
  SleepOn = false;
  stTimer = millis();
  myOLED.sleepMode(SLEEP_OFF);
  digitalWrite(RVR_Vcc_Pin, HIGH);  //включаем приемник
  Serial.print(F("wakeUP!"));
}

void loop() {
  btn_ok.tick(); btn_left.tick(); btn_right.tick();
  x_move = x_direction.read_action();
  y_move = y_direction.read_action();
  if(x_move > 10)  goright=true;
  else if(x_move < 10) goleft=true;
  else if(y_move > 10) goup=true;
  else if(y_move < 10) godown=true;
  else 
  {
    goright=false;
    goleft=false;
    goup=false;
    godown=false;
  }
  
  char echo = Serial.read(); if (echo > 0) Serial.println(echo);
  if ((echo == 'e') || (btn_left.isHold() && btn_right.isHold()))
  {
    myOLED.print(F("EEPROM cleared success!"), 0, 0);
    Serial.println(F("EEPROM cleared"));
    EEPROM.update(0, 0); EEPROM.update(1, 0);
    EEPROM_key_count = 0; EEPROM_key_index = 0;
    Sd_ReadOK();
    myOLED.update();
    stTimer = millis();
  }
  bool dcl = btn_ok.isDouble();
  if ((echo == 't') || (btn_ok.isClick() && !dcl)) {  // отправляем ключ
    sendSynthKey(&keyData1);
    Sd_WriteStep();
    stTimer = millis();
    //Serial.println("OK");
  } 
  if (goleft && (EEPROM_key_count > 0)){       //при повороте энкодера листаем ключи из eeprom
    EEPROM_key_index--;
    if (EEPROM_key_index < 1) EEPROM_key_index = EEPROM_key_count;
    EEPROM_get_key(EEPROM_key_index, &keyData1);
    OLED_printKey(&keyData1);
    Sd_WriteStep();
    stTimer = millis();
    //Serial.println("L");
    goleft=false;
  }
  if (goright && (EEPROM_key_count > 0)){
    EEPROM_key_index++;
    if (EEPROM_key_index > EEPROM_key_count) EEPROM_key_index = 1;
    EEPROM_get_key(EEPROM_key_index, &keyData1);
    OLED_printKey(&keyData1);
    Sd_WriteStep();
    stTimer = millis();
    //Serial.println("R");
    goright=false;    
  }
  if (dcl) {
    switch (snifferMode){
      case smdNormal: snifferMode = smdAutoRec; OLED_printKey(&keyData1); Sd_ReadOKK(); Sd_ReadOK(); break; 
      case smdAutoRec: Sd_ReadOKK(); Sd_ReadOK(); snifferMode = smdAutoRecSilence; myOLED.sleepMode(SLEEP_ON); break; 
      case smdAutoRecSilence: snifferMode = smdNormal; myOLED.sleepMode(SLEEP_OFF); OLED_printKey(&keyData1); Sd_ReadOKK(); Sd_ReadOK(); break; 
    }
    stTimer = millis();
    //Serial.println("2");
  }
  if ((keyData1.codeLenth != 0) &&  btn_ok.isHolded()){     // Если зажать кнопкку - ключ сохранися в EEPROM
    if (EPPROM_AddKey(&keyData1)) {
      OLED_printError(F("The key saved"), false);
      Sd_ReadOKK();
      delay(1000 >> prescal); 
    } else Sd_ErrorBeep();
    OLED_printKey(&keyData1);
    stTimer = millis();
  }
  
  if (recieved) {
    if (convert2Key(&keyData1)){
      digitalWrite(G_Led, HIGH);
      Sd_ReadOK();
      if (indxKeyInROM(&keyData1) == 0) OLED_printKey(&keyData1, 1);
        else OLED_printKey(&keyData1, 3);
      for (byte i = 0; i < keyData1.codeLenth >> 3; i++) {
        Serial.print(keyData1.keyID[i], HEX); Serial.print(" ");
        btn_ok.tick(); btn_left.tick(); btn_right.tick();
      }
    } else Sd_ErrorBeep();
    if (snifferMode != smdAutoRecSilence) printDebugData();
    if ((snifferMode != smdNormal) && (keyData1.codeLenth != 0)){
      if (EPPROM_AddKey(&keyData1)) {
        OLED_printError(F("The key saved"), false);
        Sd_ReadOKK();
        delay(500 >> prescal); 
      } else Sd_ErrorBeep();
      OLED_printKey(&keyData1);
    }
    stTimer = millis();
    recieved = false;
    digitalWrite(G_Led, LOW);
  }
  if ((millis() - stTimer > (10000 >> prescal)) && (snifferMode == smdNormal)) go2sleep(); //засыпаем через 10 сек
}

//***************** звуки****************
void Sd_ReadOK() {  // звук ОК
  if (snifferMode == smdAutoRecSilence) return;
//  for (int i=400; i<6000; i=i*1.5) { 
//    tone(speakerPin, i); delay(16 >> prescal); 
//     btn_ok.tick(); btn_left.tick(); btn_right.tick();
//  }
//  noTone(speakerPin);
  for(int i=2; i>0; i--)
  {
    digitalWrite(grn_led_pin, HIGH);
    delay(200);
    digitalWrite(grn_led_pin, LOW);
    delay(200);
   }

}

void Sd_ReadOKK() {  // звук ОК
  if (snifferMode == smdAutoRecSilence) return;
//  for (int i=800; i<5000; i=i*1.5) {
//    tone(speakerPin, i); delay(20 >> prescal); 
//    btn_ok.tick(); btn_left.tick(); btn_right.tick();
//  }
//  noTone(speakerPin);
  for(int i=1; i>0; i--)
  {
    digitalWrite(grn_led_pin, HIGH);
    delay(1000);
    digitalWrite(grn_led_pin, LOW);
    delay(10);
   }

}

void Sd_WriteStep(){  // звук "очередной шаг"
  if (snifferMode == smdAutoRecSilence) return;
//  for (int i=2500; i<6000; i=i*1.5) { 
//    tone(speakerPin, i); delay(8 >> prescal); 
//    btn_ok.tick(); btn_left.tick(); btn_right.tick();
//  }
//  noTone(speakerPin);
  for(int i=1; i>0; i--)
  {
    digitalWrite(white_led_pin, HIGH);
    delay(200);
    digitalWrite(white_led_pin, LOW);
    delay(10);
   }
}

void Sd_ErrorBeep() {  // звук "ERROR"
  if (snifferMode == smdAutoRecSilence) return;
//  for (int j=0; j <3; j++){
//    for (int i=1000; i<2000; i=i*1.1) { tone(speakerPin, i); delay(8 >> prescal); }
//    delay(40 >> prescal);
//    for (int i=1000; i>500; i=i*1.9) { tone(speakerPin, i); delay(8 >> prescal); }
//    delay(40 >> prescal);
//  }
//  noTone(speakerPin);
  for(int i=3; i>0; i--)
  {
    digitalWrite(red_led_pin, HIGH);
    delay(200);
    digitalWrite(red_led_pin, LOW);
    delay(100);
   }
}

void Sd_StartOK(){   // звук "Успешное включение"
  if (snifferMode == smdAutoRecSilence) return;
//  tone(speakerPin, NOTE_A7); delay(80 >> prescal);
//  tone(speakerPin, NOTE_G7); delay(80 >> prescal);
//  tone(speakerPin, NOTE_E7); delay(80 >> prescal); 
//  tone(speakerPin, NOTE_C7); delay(80 >> prescal);  
//  tone(speakerPin, NOTE_B7); delay(80 >> prescal);
//  tone(speakerPin, NOTE_D7); delay(80 >> prescal); 
//  tone(speakerPin, NOTE_C7); delay(80 >> prescal);
//  tone(speakerPin, NOTE_F7); delay(80 >> prescal); 
//  noTone(speakerPin); 
  for(int i=3; i>0; i--)
  {
    digitalWrite(red_led_pin, HIGH);
    delay(100);
    digitalWrite(grn_led_pin, HIGH);
    delay(100);
    digitalWrite(white_led_pin, HIGH);
    delay(100);
    digitalWrite(G_Led, HIGH);
    delay(100);
    digitalWrite(red_led_pin, LOW);
    delay(100);
    digitalWrite(grn_led_pin, LOW);
    delay(100);
    digitalWrite(white_led_pin, LOW);
    delay(100);
    digitalWrite(G_Led, LOW);
    delay(10);
   }

}
