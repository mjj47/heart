#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define TFT_DC 9
#define TFT_CS 10

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

volatile uint32_t PDB_CONFIG;
const int QUEUE_LENGTH = 321;

const int HEART_INPUT = 14;
uint32_t heartVals[QUEUE_LENGTH];
uint16_t startPoint = 0;
uint16_t endPoint = 0;
uint16_t lineDelta;
uint16_t xPos = 0;
volatile uint16_t queueSize = 0;
uint16_t blankSpace = 0;

uint32_t peekEndQueue() { 
  int index = endPoint - 1;
  index = (index >= 0) ? index : index + QUEUE_LENGTH;
  return heartVals[index];
}


uint32_t peekQueue() {
  return heartVals[startPoint];
}

uint32_t removeQueue() {
    queueSize--;
    uint32_t temp = heartVals[startPoint];
    startPoint +=1;
    startPoint = startPoint % QUEUE_LENGTH;
    return temp;
}
 
void addQueue(uint32_t val) {
  queueSize++;
  heartVals[endPoint] = val;
  endPoint += 1;
  endPoint = endPoint % QUEUE_LENGTH;
}


  
void setup() {
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);

  pinMode(HEART_INPUT, INPUT);
  lineDelta = tft.width() / (QUEUE_LENGTH - 1);
  SIM_SCGC6 |= SIM_SCGC6_PDB;
  PDB0_MOD = 37500 / 250;
  PDB0_IDLY = 0;
  
  PDB_CONFIG = PDB0_SC = PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE | PDB_SC_CONT | PDB_SC_PRESCALER(7) | PDB_SC_MULT(1);
  PDB0_SC |= PDB_SC_SWTRIG;
  PDB0_SC |= PDB_SC_LDOK;
  addQueue(0);
 
  
  NVIC_ENABLE_IRQ(IRQ_PDB);
}

void loop() {}

void pdb_isr() {
  uint32_t val = analogRead(HEART_INPUT);
  uint32_t yVal = tft.height() * val / 1023;
  if (queueSize == QUEUE_LENGTH) {
    uint32_t oldVal = removeQueue();
    uint32_t oldVal2 = peekQueue();
    uint16_t xStart = xPos + lineDelta;
    if (xStart > tft.width()) {
      xStart = 0;
    }
    
    tft.drawLine(xStart, oldVal, xStart + lineDelta,
                  oldVal2, ILI9341_BLACK);
    Serial.print("Remove Line ");
    Serial.print(xPos);
    Serial.print(",");
    Serial.print(oldVal);
    Serial.print(" ");
    Serial.print(xPos + lineDelta);
    Serial.print(",");
    Serial.print(oldVal2);
    Serial.println(" ");
    
  } 
  uint32_t temp = peekEndQueue();
  tft.drawLine(xPos, temp, xPos + lineDelta, yVal, ILI9341_GREEN);
  Serial.print("Add Line ");
  Serial.print(xPos);
  Serial.print(",");
  Serial.print(temp);
  Serial.print(" ");
  Serial.print(xPos + lineDelta);
  Serial.print(",");
  Serial.print(yVal);
  Serial.println(" ");
  addQueue(yVal);
  xPos += lineDelta;
  if (xPos > tft.width()) {
    xPos = 0;
  }
  PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
}
