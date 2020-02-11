/*
  Скетч к проекту "Универсальный пульт для шлагаумов и люср RF 433.96MHz / 315MHz с OLED дисплеем и хранением 30 ключей в памяти EEPROM"
  Аппаратная часть построена на Arduino Pro Mini 3.3v
  Исходники на GitHub: https://github.com/AlexMalov/RadioSniffer/
  Автор: МЕХАТРОН DIY, AlexMalov, 2020
  v 1.0
*/

#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <OLED_I2C.h>
#include "pitches.h"
#include "GyverButton.h"

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
#define G_Led 13        // всроенный свеодиод
#define Btn_ok_Pin 3    // Кнопка оправки текущего кода
#define Btn_left_Pin 5  // Кнопка влево
#define Btn_right_Pin 6 // Кнопка вправо

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
    if (abs(delta) < (200 >> prescal)) { // это точно повторный сигнал
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
  char echo = Serial.read(); if (echo > 0) Serial.println(echo);
  if ((echo == 'e') || (btn_left.isHold() && btn_right.isHold())){
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
  if (btn_left.isClick() && (EEPROM_key_count > 0)){       //при повороте энкодера листаем ключи из eeprom
    EEPROM_key_index--;
    if (EEPROM_key_index < 1) EEPROM_key_index = EEPROM_key_count;
    EEPROM_get_key(EEPROM_key_index, &keyData1);
    OLED_printKey(&keyData1);
    Sd_WriteStep();
    stTimer = millis();
    //Serial.println("L");
  }
  if (btn_right.isClick() && (EEPROM_key_count > 0)){
    EEPROM_key_index++;
    if (EEPROM_key_index > EEPROM_key_count) EEPROM_key_index = 1;
    EEPROM_get_key(EEPROM_key_index, &keyData1);
    OLED_printKey(&keyData1);
    Sd_WriteStep();
    stTimer = millis();
    //Serial.println("R");    
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
  

/*
  switch (copierMode){
      case md_empty: case md_read: 
        if (searchCyfral() || searchMetacom() || searchEM_Marine() || searchIbutton() ){     // запускаем поиск cyfral, затем поиск EM_Marine, затем поиск dallas
          Sd_ReadOK();
          copierMode = md_read;
          digitalWrite(G_Led, HIGH);
          if (indxKeyInROM(keyID) == 0) OLED_printKey(keyID, 1);
            else OLED_printKey(keyID, 3);
          } 
        break;
      case md_write:
        if (keyType == keyEM_Marine) write2rfid();
          else write2iBtn(); 
        break;
      case md_blueMode: 
        BM_SendKey(keyID);
        break;
    } //end switch
*/
    

/*
   tpKeyData keyData2 = {     // ворота 1
      {0, 0, 0, 0, 0, 0, 0, 0},
      {-17248, 468, -296, 468, -304, 456, -308, 452, -320, 444, -324, 440, -328, 436, -332, 432, -340, 424, -340, 424, -344, 420, -348, 416, -4232, 820, -348, 816, -356, 408, -756, 808, -360, 808, -360, 800, -368, 800, -368, 792, -372, 796, -372, 792, -376, 392, -772, 792, -380, 784, -380, 384, -784, 784, -380, 784, -384, 384, -784, 780, -388, 376, -784, 384, -784, 380, -784, 780, -392, 776, -388, 376, -788, 380, -788, 776, -392, 376, -788, 376, -792, 376, -788, 772, -396, 772, -392, 372, -796, 772, -396, 368, -796, 772, -396, 768, -400, 764, -400, 764, -404, 764, -400, 368, -800, 764, -400, 764, -404, 764, -404, 360, -804, 364, -800, 764, -404, 364, -800, 364, -804, 364, -800, 368, -796, 368, -800, 764, -404, 364, -800, 364, -800, 368, -796, 768, -400, 768, -400, 764, -404, 364, -800, 764, -404, 760, -404, 364, -804, 760, -404, 764, -408, 356, -800, 408, 0, 0, 0, 0},
      {0, 0},
      {0, 0},
      64,
      156
    };
    
    tpKeyData keyData3 = { // ворота 2
      {0, 0, 0, 0, 0, 0, 0, 0},
      {-17248, 484, -312, 480, -320, 472, -332, 464, -336, 456, -340, 456, -348, 448, -348, 444, -356, 440, -360, 436, -364, 436, -364, 432, -4232, 820, -368, 812, -372, 424, -756, 808, -380, 800, -380, 800, -384, 796, -388, 792, -392, 788, -392, 788, -396, 404, -776, 788, -396, 784, -400, 400, -780, 784, -400, 780, -400, 400, -784, 396, -784, 400, -780, 780, -408, 776, -404, 392, -792, 776, -404, 776, -408, 388, -792, 392, -792, 388, -792, 772, -412, 772, -408, 388, -796, 772, -408, 772, -416, 764, -416, 380, -800, 768, -416, 764, -420, 764, -416, 764, -420, 764, -416, 380, -800, 768, -416, 760, -420, 764, -420, 380, -804, 376, -800, 764, -420, 380, -804, 376, -808, 376, -800, 380, -804, 380, -800, 764, -420, 380, -800, 380, -804, 380, -796, 768, -420, 760, -424, 756, -424, 376, -804, 760, -424, 760, -424, 376, -804, 760, -424, 756, -428, 372, -804, 416, 0, 0, 0, 0},
      {0, 0},
      {0, 0},
      64,
      156
    }; 
*/
 /*   
    tpKeyData keyData2 = {  // люстра E
      {0, 0, 0, 0, 0, 0, 0, 0},
      {8156, -416, 668, -964, 94, -412, 680, -944, 124, -412, 696, -912, 132, -372, 716, -376, 676, -904, 140, -952, 100, -452, 648, -912, 188, -880, 128, -936, 176, -904, 152, -888, 148, -400, 720, -336, 720, -348, 708, -900, 228, -296, 748, -888, 144, -356, 732, -348, 732, -420, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0},
      {0, 0},
      24,
      50
    }; */

/*    tpKeyRawData keyData2 = {  // люстра D
      {0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0},
      {0, 0},
      {0, 0},
      {0, 0},
      {0, 0},
      0,      
      24,
      0,
      k24bt,
      50,
      {8116, -488, 588, -884, 172, -408, 648, -960, 96, -424, 676, -896, 152, -416, 652, -388, 652, -956, 120, -960, 104, -412, 676, -896, 148, -920, 152, -884, 200, -880, 172, -868, 200, -336, 744, -372, 700, -328, 724, -888, 148, -368, 736, -312, 728, -352, 680, -904, 72, -464, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };
  */  

}

//***************** звуки****************
void Sd_ReadOK() {  // звук ОК
  if (snifferMode == smdAutoRecSilence) return;
  for (int i=400; i<6000; i=i*1.5) { 
    tone(speakerPin, i); delay(16 >> prescal); 
     btn_ok.tick(); btn_left.tick(); btn_right.tick();
  }
  noTone(speakerPin);
}

void Sd_ReadOKK() {  // звук ОК
  if (snifferMode == smdAutoRecSilence) return;
  for (int i=800; i<5000; i=i*1.5) {
    tone(speakerPin, i); delay(20 >> prescal); 
    btn_ok.tick(); btn_left.tick(); btn_right.tick();
  }
  noTone(speakerPin);
}

void Sd_WriteStep(){  // звук "очередной шаг"
  if (snifferMode == smdAutoRecSilence) return;
  for (int i=2500; i<6000; i=i*1.5) { 
    tone(speakerPin, i); delay(8 >> prescal); 
    btn_ok.tick(); btn_left.tick(); btn_right.tick();
  }
  noTone(speakerPin);
}

void Sd_ErrorBeep() {  // звук "ERROR"
  if (snifferMode == smdAutoRecSilence) return;
  for (int j=0; j <3; j++){
    for (int i=1000; i<2000; i=i*1.1) { tone(speakerPin, i); delay(8 >> prescal); }
    delay(40 >> prescal);
    for (int i=1000; i>500; i=i*1.9) { tone(speakerPin, i); delay(8 >> prescal); }
    delay(40 >> prescal);
  }
  noTone(speakerPin);
}

void Sd_StartOK(){   // звук "Успешное включение"
  if (snifferMode == smdAutoRecSilence) return;
  tone(speakerPin, NOTE_A7); delay(80 >> prescal);
  tone(speakerPin, NOTE_G7); delay(80 >> prescal);
  tone(speakerPin, NOTE_E7); delay(80 >> prescal); 
  tone(speakerPin, NOTE_C7); delay(80 >> prescal);  
  tone(speakerPin, NOTE_B7); delay(80 >> prescal);
  tone(speakerPin, NOTE_D7); delay(80 >> prescal); 
  tone(speakerPin, NOTE_C7); delay(80 >> prescal);
  tone(speakerPin, NOTE_F7); delay(80 >> prescal); 
  noTone(speakerPin); 
}
