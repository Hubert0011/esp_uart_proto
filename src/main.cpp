#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <SPI.h>

#define LED 2
#define BUILTIN_LED 48
#define OLED_SCK 42
#define OLED_SDA 41
#define OLED_RES 40
#define OLED_DC 39
#define OLED_CS 38
#define UART_RX_PIN 6
#define UART_TX_PIN 7
#define LED_ERROR 3

//piny przyciskow sterujacych
#define NEXT_BTN 12
#define PREV_BTN 11
#define SELECT_BTN 13

//czas czytany jako dlugie klikniecie
#define LONG_PRESS_TIME 600



//czas na eliminacja drgan stykow
const unsigned long DEBOUNCE_MS = 140;

volatile unsigned long pressStartTime = 0;
volatile bool shortPressDetected = false;
volatile bool longPressDetected = false;

//enum ze wszystkimi stanami obecnego wyswietlania, wykorzystane przez maszyne stanow
enum oledScreenState{
  MAIN_MENU,
  LIVE_SINGLE,
  LIVE_ALL,
  FLASH_MENU
};

//struktura danej informacji
struct OBDParam
{
  uint8_t PID;
  const char* name;
  const char* unit;
  uint16_t maxVal;
};

struct options
{
  uint8_t ID;
  const char* name;
};

const options optionsList[] =
{
  {0, "Parametry I"},
  {1, "Parametry II"},
  {2, "ECU flash"}
};

//lista parametrow
const OBDParam paramList[] = 
{
  {0x0C, "RPM", "rev/min", 7000},
  {0x04, "LOAD", "%", 100},
  {0x05, "COOLANT", "C", 120},
  {0x0B, "MAF", "g/s", 250},
  {0x11, "TPS", "%", 100}
};

//liczba parametrow
//podzielony rozmiar listy przez rozmiar elementow, poniewaz sizeof zwraca wartosc w bajtach zajetych, dla wszystkich elementow w sumie
//trzeba podzielic zeby uzyskac czysty rozmiar listy
const uint8_t paramCount = sizeof(paramList) / sizeof(paramList[0]);
const uint8_t optionsCount = sizeof(optionsList) / sizeof(optionsList[0]); 

//liczba ostatnich klikniec eliminacji odbic stykow
volatile unsigned long lastPrev = 0;
volatile unsigned long lastNext = 0;
volatile unsigned long lastSelect = 0;

oledScreenState currentScreen = MAIN_MENU; 


U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(
    U8G2_R0, 
    OLED_CS, 
    OLED_DC, 
    OLED_RES
  );

void wyslijRamke(uint8_t pid)
{
  uint8_t ramka[4];
  ramka[0] = 0xAA;
  ramka[1] = 0x01;
  ramka[2] = pid;
  ramka[3] = 0x55;

  Serial1.write(ramka, 4);
}


//zmienne dla przerwan od sterowania przyciskami
volatile bool changeFlag = false;
volatile int8_t paramsCurrentIndex = 0;
volatile int8_t optionsCurrentIndex = 0;
volatile bool selected = false;
//volatile uint8_t selctedIndex = 0;

//funckje osblugi przerwan

//nacisnieto prev
void ARDUINO_ISR_ATTR isrPrev()
{
  unsigned long now = millis();
  if(now - lastPrev > DEBOUNCE_MS)
  {
    //czas na eliminacje skutkow odbic
    digitalWrite(LED_ERROR, HIGH);
    lastPrev = now;
    //przelaczamy indeksy tablicy params tylko wtedy kiedy wybralismy opcje wyswietlania parametrow
    //inaczej tylko zmieniamy indeksy tablicy z opcjami
    if(selected == true)
    {
      paramsCurrentIndex--;
      if(paramsCurrentIndex < 0)
      {
        paramsCurrentIndex = paramCount - 1 ;
      }
    }
    else
    {
    optionsCurrentIndex--;
      if(optionsCurrentIndex < 0)
      {
        optionsCurrentIndex = optionsCount - 1 ;
      }
    }
    changeFlag = true;
    digitalWrite(LED_ERROR, LOW);
  }
}

//nacisnieto next
void ARDUINO_ISR_ATTR isrNext()
{
  unsigned long now = millis();
  if(now - lastNext > DEBOUNCE_MS)
  {
    //czas na eliminacje skutkow odbic
    digitalWrite(LED_ERROR, HIGH);
    lastNext = now;
    //przelaczamy indeksy tablicy params tylko wtedy kiedy wybralismy opcje wyswietlania parametrow
    //inaczej tylko zmieniamy indeksy tablicy z opcjami
    if(selected == true)
    {
      paramsCurrentIndex++;
      if(paramsCurrentIndex >= 4)
      {
        paramsCurrentIndex = 0 ;
      }
    }
    else{
    optionsCurrentIndex++;
      if(optionsCurrentIndex >= 3)
      {
        optionsCurrentIndex = 0 ;
      }
    }
    changeFlag = true;
    digitalWrite(LED_ERROR, LOW);
  }
}

//nacisnieto select
void ARDUINO_ISR_ATTR isrSelect()
{
  unsigned long now = millis();
  bool pinState = digitalRead(SELECT_BTN);
  
  if(pinState == LOW)
  {
    //moment wcisniecia przycisku
    pressStartTime = now;
    digitalWrite(LED_ERROR, HIGH);
  }
  else
  {
    //wykryto puszczenie przycisku
    if(pressStartTime > 0)
    {
      unsigned long pressDuration = now - pressStartTime;
      if(pressDuration >= LONG_PRESS_TIME)
      {
        longPressDetected = true;
      }
      else
      {
        shortPressDetected = true;
        digitalWrite(LED_ERROR, LOW);
      }
      pressStartTime = 0;
    }
  }
}

volatile unsigned long last = 0;

int8_t i = 0;
int16_t input = 0;
uint16_t receivedValue = 0;
unsigned long start = millis();

//funckje wyswietlacjace odpowiednie wartosci

void displaySingleParam(uint8_t activePID, const char* name, const char* unit, uint16_t maxVal)
{

  selected = true;
//Odpytujemy ECU co 25 ms, aktywuje sie kiedy wykryje przerwanie, pobierze nowe PID, i zapyta atmege o nowe PID
      if(changeFlag == true){ 
        digitalWrite(LED_ERROR, HIGH);
        changeFlag = false;
        digitalWrite(LED_ERROR, LOW);

      }
      //odczytanie czystej, pelnej ramki
      if (millis() - start >= 25)
      {
        start = millis();
        wyslijRamke(activePID);
      }

      while(Serial1.available() >= 4)
      {
        // Szukamy nagłówka 0xBB
        if (Serial1.peek() != 0xBB)
        {
          Serial1.read(); // Wyrzucamy pojedynczy przesunięty bajt
          continue;
        }
          uint8_t naglowek = Serial1.read();
          uint8_t pid = Serial1.read();
          uint8_t bajtA = Serial1.read(); //bajt danych
          uint8_t bajtB = Serial1.read(); //bajt danych

          if(pid == activePID)
          {
            if(pid == 0x0C)
            {
              receivedValue = (bajtA << 8) | bajtB;
              if(receivedValue > 6300) digitalWrite(LED_ERROR, HIGH);
              else digitalWrite(LED_ERROR, LOW);
            }
            else
            {
              receivedValue = bajtB;
            }
          }
          
          Serial.printf("0x%02X 0x%02X 0x%02X 0x%02X | \r", naglowek, pid, bajtA, bajtB);

          //wyswietlanie na ekranie OLED
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_7x14B_tr);
          u8g2.drawStr(8, 10, "Value: ");
          u8g2.drawStr(51, 10, name);
          u8g2.setFont(u8g2_font_5x7_tr);
          u8g2.drawStr(95, 30, "MS41.0");

          u8g2.setFont(u8g2_font_logisoso24_tr);
          u8g2.setCursor(10, 48);
          u8g2.print(receivedValue);

          u8g2.setFont(u8g2_font_6x12_tr);
          u8g2.drawStr(75, 48, unit);
          u8g2.drawFrame(10, 54, 108, 6);
          int szerokosc = map(receivedValue, 0, maxVal, 0, 104);
          if (szerokosc > 104) szerokosc = 104;
          u8g2.drawBox(12, 56, szerokosc, 2);
          u8g2.sendBuffer();
      }
}

void displayMenu()
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14_mr);
    u8g2.drawStr(6, 12, "SIEMENS MS41.0");
    u8g2.setCursor(6, 47);
    u8g2.print(optionsCurrentIndex);
    u8g2.setFont(u8g2_font_6x10_mr);
    u8g2.drawStr(8, 25, "OPCJE-> ");

    for(uint8_t i = 0; i < optionsCount; i++)
    {

      if(i == optionsCurrentIndex)
      {
        //ustawienie bialego tla na zaznaczony napis
        u8g2.setDrawColor(1);
        u8g2.drawBox(50, 16 + (i*12), 76, 11);

        //ustawienie pisania na czarno na bialym tle
        u8g2.setDrawColor(0);
        u8g2.drawStr(53, 25 + (i*12), optionsList[i].name);

        //przywrocenie koloru bialego tekstu
        u8g2.setDrawColor(1);
      }
      else
      {
        u8g2.drawStr(53, 25 + (i*12), optionsList[i].name);
      }

    }
    u8g2.sendBuffer();
}

void dmeCommunication()
{
  Serial1.write(0xAA);
  delay(500);
  while(Serial1.available()) {
    Serial.printf("Echo: %02X\n", Serial1.read());
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(UART_RX_PIN, INPUT_PULLUP);

  //konfiguracja pinow od przerwan
  pinMode(PREV_BTN, INPUT_PULLUP);
  pinMode(NEXT_BTN, INPUT_PULLUP);
  pinMode(SELECT_BTN, INPUT_PULLUP);
  
  //konfiguracja obslugi przerwan
  attachInterrupt(digitalPinToInterrupt(PREV_BTN), isrPrev, FALLING);
  attachInterrupt(digitalPinToInterrupt(NEXT_BTN), isrNext, FALLING);
  attachInterrupt(digitalPinToInterrupt(SELECT_BTN), isrSelect, CHANGE);


  //delay(1000);
  Serial1.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  //uruchomienie sprzetowego SPI
  SPI.begin(OLED_SCK, -1, OLED_SDA, OLED_CS);
  
  //uruchomiemie ekranu
  u8g2.begin();
  u8g2.setBusClock(10000000);
  u8g2.setContrast(255); // Maksymalna jasność

  delay(1500);

  pinMode(LED_ERROR, OUTPUT);

  while (Serial1.available()) {
        Serial1.read();
    }
        
}

void loop()
{
  uint8_t activePID = paramList[paramsCurrentIndex].PID;
  const char* name = paramList[paramsCurrentIndex].name;
  const char* unit = paramList[paramsCurrentIndex].unit;
  uint16_t maxVal = paramList[paramsCurrentIndex].maxVal;

  uint8_t optionID = optionsList[optionsCurrentIndex].ID;
  const char* optionName = optionsList[optionsCurrentIndex].name;


  dmeCommunication();
  //maszyna stanow, sterowanie selectem
  if(longPressDetected)
  {
    longPressDetected = false;
    selected = false;
    if(currentScreen != MAIN_MENU)
    {
      currentScreen = MAIN_MENU;
    }
  }
  if(shortPressDetected)
  {
    shortPressDetected = false;
    if(currentScreen == MAIN_MENU)
    {
      switch(optionsCurrentIndex)
      {
        case 0: 
          currentScreen = LIVE_SINGLE;
          break;
        case 1: 
          currentScreen = LIVE_ALL;
          break;
        case 2: 
          currentScreen = FLASH_MENU;
          break;
      }
    }
  }

  switch(currentScreen)
  {
    case MAIN_MENU: 
      displayMenu();
      break;
    case LIVE_SINGLE: 
      displaySingleParam(activePID, name, unit, maxVal);
      break;
    case LIVE_ALL:
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_7x14B_tr);
      u8g2.drawStr(8, 10, "Parametry II");
      u8g2.sendBuffer();
      break;
    case FLASH_MENU:
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_7x14B_tr);
      u8g2.drawStr(8, 10, "Flasher MENU");
      u8g2.sendBuffer();
      break;
  }

}
