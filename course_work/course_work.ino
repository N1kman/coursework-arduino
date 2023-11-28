//библиотека для ИК приемника
#include <NecDecoder.h> 
//библиотека для датчика температура
#include <Adafruit_AHTX0.h>  
//библиотека для lcd дисплея                   
#include <LiquidCrystal_I2C.h>
//библиотека для gps датчика
#include <TinyGPS++.h>
//библиотека для порта передачи с gps датчиком
#include <SoftwareSerial.h>
//библиотека для часов реального времени
#include <iarduino_RTC.h>
//библиотека для EEPROM
#include <EEPROM.h>
 
#define _RX 0        //вывод RX ардуино         
#define _TX 1        //вывод TX ардуино        
#define _PIN_13 13   //вывод D13 ардуино 
#define _PIN_12 12   //вывод D12 ардуино 
#define _PIN_11 11   //вывод D11 ардуино 
#define _PIN_10 10   //вывод D10 ардуино 
#define _PIN_8 8     //вывод D8 ардуино 
#define _PIN_4 4     //вывод D4 ардуино 
#define _PIN_3 3     //вывод D3 ардуино 

#define _INT0 0      //прерывание 0
#define _INT1 1      //прерывание 1

//скорость передачи порта
#define speedGPSPort 9600

//количество стартовых сигналов
#define startSygnals 3
//стартовая задержка
#define startDelay 400

//стартовая скорость ограничения
#define startControlSpeed 25
//нижняя граница скорости ограничения
#define lowSpeed 10
//верхняя граница скорости ограничения
#define highSpeed 40

//размер колес
#define size26 "26"
#define size27_5 "27.5"
#define size29 "29"

//длины окружности колес
#define length26 2.1
#define length27_5 2.2
#define length29 2.3

//значения для вывода информации
#define time 0        //вывод времени
#define speedDir 1    //вывод скорости и направления
#define speedParams 2 //вывод параметров скорости
#define tempHum 3     //вывод температуры и влажности
#define distance 4    //вывод расстояний 

//окно коллизий
#define windowCollision 80

//адреса данных в EEPROM
#define addressAllDist 0   //адрес всего расстояния
#define addressTodayDist 8 //адрес сегоднешнего расстояния
#define addressDay 16      //адрес дня последнего измерения
#define addressMonth 24    //адрес месяца последнего измерения

//задержки 
#define delay1sec 1000 // 1 секунда
#define delay2sec 2000 // 2 секунда
#define delay4sec 4000 // 4 секунда

//частота пьезодинамика
#define freq_2000 2000

//количество цифр в числе
#define have1Digit 10  //1 цифра
#define have2Digit 100 //2 цифры

//коды кнопок
#define button_0 48
#define button_1 48
#define button_2 24
#define button_3 122
#define button_minus 224
#define button_plus 168
#define button_pause 194
#define button_ch1 2
#define button_ch2 98
#define button_ch3 34
#define button_ch4 162
#define button_ch5 226

//структура времени
typedef struct timeStruct {
    int hour; //часы
    int min;  //минуты
    int sec;  //секунды
};

//структура для хранения времени
timeStruct tm;

//gps датчик
TinyGPSPlus gps;
//порт для соединения к gps
SoftwareSerial SoftSerial(_RX, _TX);

//lcd дисплей
LiquidCrystal_I2C lcd(0x27, 20, 4);

//часы реального времени
iarduino_RTC clock(RTC_DS1302, _PIN_12, _PIN_8, _PIN_10);

//ИК приемник
NecDecoder ir;

//датчик температуры
Adafruit_AHTX0 aht;
//значения измерений
sensors_event_t humidity, temp;

//скорость ограничения
int speedControl = startControlSpeed;
//флаг контроля скорости
volatile bool flagControlSpeed = true;
//скорость
int speedKmh;

//последнее измерение скорости
unsigned long lastFlash;

//последнее снятие значения с датчика температуры
unsigned long takeInfoAht;

//последняя запись в EEPROM и получение информации от gps
unsigned long writing;

//текущая длина окружности колеса
volatile float nowLength = length26;

//флаг для вывода информации
volatile int flagOut = 0;
//значение до изменения флага вывода
int lastFlagOut = 0;

//значение времени
unsigned long mill;

//расстояние за поездку
volatile double travelDist = 0;
//все расстояние
volatile double allDist = 0;
//расстояние за сегодня
volatile double todayDist = 0;

int day;    //день
int month;  //месяц

//флаг очистки расстояний
bool flagClearDist = false;

//функция перевода секунд во время
void secondsToTime(unsigned long seconds) {
  tm.sec = seconds % 60;           //получение секунд
  tm.min = (seconds / 60) % 60;    //получение минут
  tm.hour = (seconds / 3600) % 24; //получение часов
}

//функция обработки нажатия клавиш
void irIsr() {
  ir.tick(); //начать обработку
  //если доступно
  if (ir.available()) {
    //чтение значения кнопки
    int value = ir.readCommand();  
    
    //если нажата кнопка 1 и установлен флаг вывода параметров скорости
    if(value == button_1 && flagOut == speedParams) { 
      nowLength = length26;   //изменить размер колеса
    }
    //если нажата кнопка 2 и установлен флаг вывода параметров скорости
    if(value == button_2 && flagOut == speedParams) {
      nowLength = length27_5; //изменить размер колеса
    }
    //если нажата кнопка 3 и установлен флаг вывода параметров скорости
    if(value == button_3 && flagOut == speedParams) { 
      nowLength = length29;   //изменить размер колеса
    }

    //если нажата кнопка минус, контролируемая скорость больше минимальной
    // и установлен флаг вывода параметров скорости
    if(value == button_minus && speedControl > lowSpeed 
                             && flagOut == speedParams) {
      speedControl--; //уменьшение контролируемой скорости
    }
    //если нажата кнопка плюс, контролируемая скорость меньше максимальной
    // и установлен флаг вывода параметров скорости
    if(value == button_plus && speedControl < highSpeed 
                            && flagOut == speedParams) { 
      speedControl++; //увеличение контролируемой скорости
    }

    //если нажата кнопка канал1
    if(value == button_ch1) {
      flagOut = speedDir; //установить вывод скорости и направления
    }
    //если нажата кнопка канал2
    if(value == button_ch2) {
      flagOut = tempHum;   //установить вывод температуры и влажности
    }
    //если нажата кнопка канал3
    if(value == button_ch3) {
      flagOut = speedParams;//установить вывод параметров скорости
    }
    //если нажата кнопка канал4
    if(value == button_ch4) {
      flagOut = time;       //установить вывод времени
    }
    //если нажата кнопка канал5
    if(value == button_ch5) {
      flagOut = distance;   //установить вывод расстояний
    }

    //если нажата кнопка 0 и флаг вывода установлен на расстояние
    if(value == button_0 && flagOut == distance) {
      //обнуление всех расстояний
      allDist = 0;
      todayDist = 0;
      travelDist = 0;
      //установка флага обнуления расстояний
      flagClearDist = true;
    }

    //если нажата кнопка пауза и установлен флаг вывода
    //параметров скорости
    if(value == button_pause && flagOut == speedParams) {
      if(flagControlSpeed){
        flagControlSpeed = false; //сброс флага котроля скорости
      } else {
        flagControlSpeed = true;  //установка флага котроля скорости
      }
    }
  }
}

//функция измерения скорости
void speed() {
  //сохранение времени измерения
  mill = millis();
  //ожидание окна коллизий
  if(mill - lastFlash > windowCollision){
    //вычисление скорости
    speedKmh = (nowLength / (((float)(mill - lastFlash)) / 1000)) * 3.6;
    //сохранение последнего времени измерения
    lastFlash = mill;
    //увеличение расстояний
    travelDist += nowLength;
    allDist += nowLength;
    todayDist += nowLength;
  }
}

//функция вывода температуры и влажности
void outTempHum() {
  //установка курсора в позицию 0 0
  lcd.setCursor(0, 0);
  //вывод температуры
  lcd.print("Temperature: "); 
  lcd.print((int)temp.temperature); 
  lcd.print(" C");
  //установка курсора в позицию 1 1
  lcd.setCursor(0, 1);
  //вывод влажности
  lcd.print("Humidity: "); 
  lcd.print((int)humidity.relative_humidity); 
  lcd.print(" %");
}

//функция вывода скорости и направления
void outSpeedDir() {
  //установка курсора в позицию 0 0
  lcd.setCursor(0, 0);
  //вывод скорости
  lcd.print("SPEED (km/h): ");
  lcd.print(speedKmh);
  lcd.print("   ");
  //установка курсора в позицию 0 1
  lcd.setCursor(0, 1);
  lcd.print("DIRACTION");
  //установка курсора в позицию 0 2
  lcd.setCursor(0, 2);
  lcd.print("degrees: ");
  //если курс определен
  if(gps.course.isValid()) {
    //получение градусов
    int deg = gps.course.deg();
    //вывод градусов
    lcd.print(deg);
    //если цифр меньше 2
    if(deg < have2Digit) {
      lcd.print(" ");   //удаление оставшейся цифры
    }
    //если цифр меньше 1
    if(deg < have1Digit) {
      lcd.print(" ");   //удаление оставшейся цифры
    }
  }
}

//функция вывод параметров скорости
void outSpeedParams() {
  //установка курсора в позицию 0 0
  lcd.setCursor(0, 0);
  lcd.print("SIZE: "); 
  //если размер 26 дюймов
  if(nowLength == length26){
    lcd.print(size26);   //вывод размера
    lcd.print("  ");
  //если размер 27.5 дюймов
  } else if(nowLength == length27_5) {
    lcd.print(size27_5); //вывод размера
  //если размер 29 дюймов
  } else {
    lcd.print(size29);   //вывод размера
    lcd.print("  ");
  }
  //установка курсора в позицию 0 1
  lcd.setCursor(0, 1);
  lcd.print("CONTROL: "); 
  //если скорость контролируется
  if(flagControlSpeed) {
    lcd.print("ON");   //вывод сообщения "включено"
    lcd.print(" ");
  //если скорость не контролируется
  } else {
    lcd.print("OFF");  //вывод сообщения "отключено"
  }
  //установка курсора в позицию 0 2
  lcd.setCursor(0, 2);
  //вывод контролируемой скорости
  lcd.print("CONTROL SPEED: "); 
  lcd.print(speedControl);
}

//функция вывода времени
void outTime() {
  //установка курсора в позицию 0 0
  lcd.setCursor(0, 0);
  //вывод текущей даты
  lcd.print("DATE: ");
  lcd.print(clock.gettime("d-m-Y"));
  //установка курсора в позицию 0 1
  lcd.setCursor(0, 1);
  //вывод текущего времени
  lcd.print("TIME: ");
  lcd.print(clock.gettime("H:i:s")); 
  //установка курсора в позицию 0 2
  lcd.setCursor(0, 2);
  //вывод времени поездки
  lcd.print("TRAVEL: "); 
  secondsToTime(millis() / 1000);
  //если число состоит 1 цифры
  if(tm.hour < have1Digit) {
    lcd.print("0");
  }
  //вывод часов
  lcd.print(tm.hour);
  lcd.print(":");
  //если число состоит 1 цифры
  if(tm.min < have1Digit) {
    lcd.print("0");
  }
  //вывод минут
  lcd.print(tm.min);
  lcd.print(":");
  //если число состоит 1 цифры
  if(tm.sec < have1Digit) {
    lcd.print("0");
  }
  //вывод секунд
  lcd.print(tm.sec);   
}

//вывод расстояний 
void outDistance() {
  //если установлен флаг очистки
  if(flagClearDist) {
    flagClearDist = false; //сброс флага очистки
    lcd.clear();           //очистка экрана
  }
  //установка курсора в позицию 0 0
  lcd.setCursor(0, 0);
  lcd.print("DISTANCE (m)");
  //установка курсора в позицию 0 1
  lcd.setCursor(0, 1);
  //вывод расстояния за все время
  lcd.print("ALL TIME: ");
  lcd.print((long)allDist);
  //установка курсора в позицию 0 2
  lcd.setCursor(0, 2);
  //вывод расстояния за сегодня
  lcd.print("TODAY: ");
  lcd.print((long)todayDist);
  //установка курсора в позицию 0 3
  lcd.setCursor(0, 3);
  //вывод расстояния за поездку
  lcd.print("NOW: "); 
  lcd.print((long)travelDist);
}

//установка начальных значений
void setup() {
  //включение порта GPS
  SoftSerial.begin(speedGPSPort);

  //установка выводов на выход
  pinMode(_PIN_13, OUTPUT);
  pinMode(_PIN_11, OUTPUT); 
  pinMode(_PIN_4, OUTPUT);

  //установка 0 прерывания (ик приемник)             
  attachInterrupt(_INT0, irIsr, FALLING);

  //установка 1 прерывания (герконовый датчик) 
  attachInterrupt(_INT1, speed, RISING);

  //запуск работы с датчиком температуры
  aht.begin();

  //запуск дисплея
  lcd.init();
  lcd.backlight();

  //цикл сигнализирования о запуске устройства
  for(int i = 0; i < startSygnals; ++i){
    //включение диода
    digitalWrite(_PIN_13, HIGH);  
    //задержка 
    delay(startDelay); 
    //выключение диода             
    digitalWrite(_PIN_13, LOW); 
    //задержка 
    delay(startDelay);  
  }      

  //запуск работы с часами реального времени
  clock.begin();
  //установка времени (при необходимости расскомментировать)
  //clock.settime(0, 14, 18, 21, 11, 23, 2);
  //получение из памяти всего пути 
  EEPROM.get(addressAllDist, allDist);
  //получение из памяти пути за день
  EEPROM.get(addressTodayDist, todayDist);
  //получение дня последнего запуска
  EEPROM.get(addressDay, day);
  //получение месяца последнего запуска
  EEPROM.get(addressMonth, month);
  //если дни не совпадают
  if(day != clock.day || month != clock.month) {
    //обнуление расстояния за сегодня
    todayDist = 0;
    //запись в память расстояния, дня и месяца
    EEPROM.put(addressTodayDist, todayDist);
    EEPROM.put(addressDay, (int)clock.day);
    EEPROM.put(addressMonth, (int)clock.month);
  }  
}

//цикл выполнения программы
void loop() { 
  //если пришло время записи в память и чтения из порта gps
  if(millis() - writing > delay1sec) {
    writing = millis(); //сохрание времени последней записи
    //сохранение всего расстояния
    EEPROM.put(addressAllDist, allDist);
    //сохранение расстояния за сегодня
    EEPROM.put(addressTodayDist, todayDist);
    //если порт доступен
    if(SoftSerial.available() > 0) {
      //декодирование данных
      gps.encode(SoftSerial.read());
    }
  }

  //если долгое время не было измерений
  if (millis() - lastFlash > delay2sec) {
    speedKmh = 0;  //обнуление скорости
  }

  //если пришло время получить вданные с датчика температуры
  if(millis() - takeInfoAht > delay4sec) {
    //сохранение времени обращения к датчику температуры
    takeInfoAht = millis();
    //получение значений с датчика
    aht.getEvent(&humidity, &temp); 
  }
  
  //фиксация превышения скорости
  if(speedKmh > speedControl && flagControlSpeed) {
    digitalWrite(_PIN_11, HIGH); //открытие реле
    tone(_PIN_4, freq_2000);     //запустили звучание
  } else  {
    digitalWrite(_PIN_11, LOW);  //закрытие реле
    noTone(_PIN_4);              //остановили звучание
  }

  //если флаг вывода был изменен
  if(flagOut != lastFlagOut) {
    lcd.clear();           //очистка экрана
    //сохранение последнего значения флага
    lastFlagOut = flagOut; 
  }

  //если флаг установлен в вывод параметров скорости
  if(flagOut == speedParams) {
    outSpeedParams();  //вывод параметров скорости
  }

  //если флаг установлен в вывод скорости и направления
  if(flagOut == speedDir) {
    outSpeedDir();     //вывод скорости и направления
  }

  //если флаг установлен в вывод температуры и влажности
  if(flagOut == tempHum) {
    outTempHum();      //вывод температуры и влажности
  }

  //если флаг установлен в вывод времени
  if(flagOut == time) {
    outTime();         //вывод времени
  }  

  //если флаг установлен в вывод расстояния
  if(flagOut == distance) {
    outDistance();     //вывод расстояния
  } 
}
