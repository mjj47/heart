#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <SdFat.h>

#define TFT_DC 9
#define TFT_CS 10

#define SD_CS 4
#define FILE_BASE_NAME "DATA"

/* ADCpdbDMA
PDB triggers the ADC which requests the DMA to move the data to a buffer
*/
#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01

#define GRAPH_BACKGROUND ILI9341_WHITE
#define GRAPH_GRID_LINES ILI9341_GREEN
#define GRAPH_LINE ILI9341_BLACK
#define MENU_BACKGROUND ILI9341_BLACK
#define CALIBRATING ILI9341_BLACK
#define MENU_HEADER ILI9341_RED
#define MENU_TEXT ILI9341_WHITE

const uint8_t chipSelect = SD_CS;
SdFat sd;
SdFile file;
char fileName[13] = FILE_BASE_NAME "00.CSV";
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

volatile uint32_t PDB_CONFIG;
const int QUEUE_LENGTH = 321;
const int HERTZ = 250;
const float TIME_GRID_DELTA = .04;

uint32_t num_vert_lines = (float) QUEUE_LENGTH / HERTZ / TIME_GRID_DELTA;
uint32_t grid_delta; 

const int HEART_INPUT = 14;
uint32_t heartVals[QUEUE_LENGTH];
uint16_t startPoint = 0;
uint16_t endPoint = 0;
uint16_t lineDelta;
uint16_t xPos = 0;
volatile uint16_t queueSize = 0;
uint16_t blankSpace = 0;
volatile boolean reading_state = false;
int prev = HIGH;
uint32_t count = 0;
boolean menu_state = true;
boolean to_menu_state = false;
uint16_t sampleNumber = 0;
volatile boolean hasData;
volatile uint32_t adcData;
const uint8_t RECORD_TIME = 30;
uint32_t sdOutput[HERTZ * RECORD_TIME];
volatile uint32_t sdIndex = 0;

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


void setFileName(uint16_t index) {
  int nums = 4;
  
  fileName[nums] = '0' + (index / 10);
  fileName[nums + 1] = '0' + (index % 10);
}

void readFile(uint16_t index) {
  setFileName(index);
  Serial.print("Avail: ");
  Serial.println(file.available());
  Serial.println("Trogdor");
  int c;
  
  file.open(fileName);
  
  while ((c = file.read()) >= 0) {
    Serial.print((char)c);
  }
  
  Serial.println("DONE!!");
  

}

void openFile(uint16_t index) {
  setFileName(index - 1);
  if (!file.open(fileName, FILE_WRITE)) {
    //error("file.open");
    Serial.println("Could Not open file");
  } else {
    Serial.print("Opened File: ");
    Serial.println(fileName);
  }
}
  
  
  
void setup() {
  tft.begin();
  tft.setRotation(1);
  pinMode(1, INPUT_PULLUP);
  pinMode(0, INPUT_PULLUP);

  
  Serial.begin(9600);
  //while (!Serial);
  
  grid_delta = tft.width() / num_vert_lines;
  lineDelta = tft.width() / (QUEUE_LENGTH - 1);
  pinMode(HEART_INPUT, INPUT);
  Serial.println("woot");
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    Serial.println("Can't write to SD card");
  } else {
    Serial.println("SD card set up");
  }
  
  adcInit();
  pdbInit();
  
  NVIC_ENABLE_IRQ(IRQ_PDB);
  initMenuState();
  
}


void initReadingDataState(uint32_t time) {
    tft.fillScreen(GRAPH_BACKGROUND);
    tft.setCursor(tft.width() / 2 - 80, tft.height() / 2);
    tft.setTextColor(CALIBRATING);
    tft.setTextSize(2);
    tft.print("Callibrating....");
    int maxV = -1;
    int minV = 1024;
    int index = 0;
    int calBuf[200];
    int calBufSize = 0;
    while(abs(maxV - minV) > 1500) {
      if(!hasData) {
        continue;
      }
      calBuf[calBufSize % 200] = adcData;
      calBufSize++;
      hasData = false;
      if(calBufSize < 200) {
        continue;
      }
      maxV = -1;
      minV = 1024;
      for(int i  = 0; i < 200; i++) {
        maxV = max(calBuf[i], maxV);
        minV = min(calBuf[i], minV);
      }
    }
    
    
    //init the graph
    tft.fillScreen(GRAPH_BACKGROUND);
    initVertLines();
    initHorLines();
    
    //reset the queue
    startPoint = 0;
    endPoint = 0;
    queueSize = 0;
    xPos = 0;
    
    //reset the sd and time
    count = time;
    sdIndex = 0;
    
    //add the first filler data point
    addQueue(0);
    
    //start reading
    menu_state = false;
    reading_state = true;
}

void initMenuState() {
  reading_state = false;
  tft.fillScreen(MENU_BACKGROUND);
  tft.setTextColor(MENU_HEADER);
  tft.setTextSize(3);
  tft.setCursor(45, 10); tft.print("EKG Monitor");
  tft.setTextSize(2);
  tft.setTextColor(MENU_TEXT);
  tft.setCursor(30, 80); tft.print("Begin reading");
  menu_state = true;
  to_menu_state = false;
  
}

void initVertLines() {
  for(int i = 0; i <= num_vert_lines; i++) {
    tft.drawLine(i * grid_delta, 0, i * grid_delta, tft.height(), GRAPH_GRID_LINES);
  }
}

void redrawHorLines(int x1, int y1, int x2, int y2) {
  int minY = min(y1, y2) - 1;
  int maxY = max(y1, y2) + 1;
  int startingLine = minY / grid_delta;
  int finishingLine = maxY / grid_delta + 1;
  for(int i = startingLine ; i <= finishingLine; i++) {
    if(i * grid_delta <= maxY && i * grid_delta >= minY) {
      tft.drawLine(x1, i * grid_delta, x2, i * grid_delta,  GRAPH_GRID_LINES);
    }
  }
}

void initHorLines() {
  for(int i = 0; i <= tft.height() / grid_delta; i++) {
    tft.drawLine(0, i * grid_delta, tft.width(), i * grid_delta, GRAPH_GRID_LINES);
  }
}

void redrawVertLines(int x1, int y1, int x2, int y2) {
  int minY = min(y1, y2);
  int maxY = max(y1, y2);
  int startingLine = x1 / grid_delta;
  int finishingLine = x2 / grid_delta + 1;
  for(int i = startingLine; i <= finishingLine; i++) {
    if(i * grid_delta <= x2 && i * grid_delta >= x1) {
      tft.drawLine(i * grid_delta, minY - 1, i * grid_delta, maxY + 1, GRAPH_GRID_LINES);
    }
  }
}



void writeToSD() { 
  openFile(sampleNumber);
  Serial.println(sdIndex);
  file.print("RMMJ");
  if (sampleNumber < 10) {
    file.print("0");
  }
  file.print(sampleNumber);
  file.print(",");
  file.println(HERTZ);
  int rows = (sdIndex - 1) / 8 + 1;
  for (int row = 0; row < rows; row++) {
    file.print(sdOutput[row * 8]);
    for (int col = 1; col < 8 && row * 8 + col < sdIndex; col++) {
      file.print(",");
      file.print(sdOutput[row * 8 + col]);
    }
    file.println();
  } 
  file.println("\0");
  file.flush();
  file.close();
  
}

int i2 = 0;

uint32_t startTime;
volatile boolean to_munu_state;

volatile int prev2;
void checkReadBack() {
  int sensorVal = digitalRead(0);

  if (prev2 != sensorVal && sensorVal == LOW) {
    readFile(sampleNumber);
  }
  prev2 = sensorVal;

}

void loop() {
  uint32_t time = millis();
  int sensorVal = digitalRead(1);
  checkReadBack();
  
  if (prev != sensorVal && sensorVal == LOW) {
    if(menu_state) {
      initReadingDataState(time);
    } else if(reading_state) {
      to_menu_state = true;    
    }
  }
  prev = sensorVal;
    
 if(to_menu_state || reading_state && time - count > 300000 || sdIndex == HERTZ * RECORD_TIME) {
   writeToSD(); 
   sampleNumber++;
   sampleNumber = sampleNumber % 100;
   sdIndex = 0;
   initMenuState();
 } 

   
  if (hasData && reading_state) {
      hasData = false;    
      uint32_t yVal = tft.height() * adcData / 4095;
      if (queueSize == QUEUE_LENGTH) {
        uint32_t oldVal = removeQueue();
        uint32_t oldVal2 = peekQueue();
        uint16_t xStart = xPos + lineDelta;
        if (xStart > tft.width()) {
          xStart = 0;
        }
        tft.drawLine(xStart, oldVal, xStart + lineDelta,
                      oldVal2, GRAPH_BACKGROUND);
        tft.drawLine(xStart, oldVal + 1, xStart + lineDelta,
                      oldVal2 + 1, GRAPH_BACKGROUND);                
        tft.drawLine(xStart, oldVal - 1, xStart + lineDelta,
                      oldVal2 - 1, GRAPH_BACKGROUND);
        redrawVertLines(xStart, oldVal, xStart + lineDelta, oldVal2);
        redrawHorLines(xStart, oldVal, xStart + lineDelta, oldVal2);
      } 
      uint32_t temp = peekEndQueue();
      tft.drawLine(xPos, temp, xPos + lineDelta, yVal, GRAPH_LINE);
      tft.drawLine(xPos, temp - 1, xPos + lineDelta, yVal - 1, GRAPH_LINE);
      tft.drawLine(xPos, temp + 1, xPos + lineDelta, yVal + 1, GRAPH_LINE);
      addQueue(yVal);
      xPos += lineDelta;
      if (xPos > tft.width()) {
        xPos = 0;        
           
      }
      sdOutput[sdIndex] = adcData;
      sdIndex++;  
  }   
}

static const uint8_t channel2sc1a[] = {
	5, 14, 8, 9, 13, 12, 6, 7, 15, 4,
	0, 19, 3, 21, 26, 22
};

/*
	ADC_CFG1_ADIV(2)         Divide ratio = 4 (F_BUS = 48 MHz => ADCK = 12 MHz)
	ADC_CFG1_MODE(2)         Single ended 10 bit mode
	ADC_CFG1_ADLSMP          Long sample time
*/
#define ADC_CONFIG1 (ADC_CFG1_ADIV(1) | ADC_CFG1_MODE(1) | ADC_CFG1_ADLSMP)

/*
	ADC_CFG2_MUXSEL          Select channels ADxxb
	ADC_CFG2_ADLSTS(3)       Shortest long sample time
*/
#define ADC_CONFIG2 (ADC_CFG2_MUXSEL | ADC_CFG2_ADLSTS(3))

void adcInit() {
	ADC0_CFG1 = ADC_CONFIG1;
	ADC0_CFG2 = ADC_CONFIG2;
	// Voltage ref vcc, hardware trigger, DMA
	ADC0_SC2 = ADC_SC2_REFSEL(0) | ADC_SC2_ADTRG;
        //ADC0_SC2 = ADC_SC2_REFSEL(0) | ADC_SC2_ADTRG | ADC_SC2_DMAEN;

	// Enable averaging, 4 samples
	ADC0_SC3 = ADC_SC3_AVGE | ADC_SC3_AVGS(0);

	adcCalibrate();
	Serial.println("calibrated");

	// Enable ADC interrupt, configure pin
	ADC0_SC1A = ADC_SC1_AIEN | channel2sc1a[3];
	NVIC_ENABLE_IRQ(IRQ_ADC0);
}

void adcCalibrate() {
	uint16_t sum;

	// Begin calibration
	ADC0_SC3 = ADC_SC3_CAL;
	// Wait for calibration
	while (ADC0_SC3 & ADC_SC3_CAL);

	// Plus side gain
	sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
	sum = (sum / 2) | 0x8000;
	ADC0_PG = sum;

	// Minus side gain (not used in single-ended mode)
	sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
	sum = (sum / 2) | 0x8000;
	ADC0_MG = sum;
}

/*
	PDB_SC_TRGSEL(15)        Select software trigger
	PDB_SC_PDBEN             PDB enable
	PDB_SC_PDBIE             Interrupt enable
	PDB_SC_CONT              Continuous mode
	PDB_SC_PRESCALER(7)      Prescaler = 128
	PDB_SC_MULT(1)           Prescaler multiplication factor = 10
*/
#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE \
	| PDB_SC_CONT | PDB_SC_PRESCALER(7) | PDB_SC_MULT(1))

// 48 MHz / 128 / 10 / 1 Hz = 37500
#define PDB_PERIOD (F_BUS / 128 / 10 / HERTZ)

void pdbInit() {

	// Enable PDB clock
	SIM_SCGC6 |= SIM_SCGC6_PDB;
	// Timer period
	PDB0_MOD = PDB_PERIOD;
	// Interrupt delay
	PDB0_IDLY = 0;
	// Enable pre-trigger
	PDB0_CH0C1 = PDB_CH0C1_TOS | PDB_CH0C1_EN;
	// PDB0_CH0DLY0 = 0;
	PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
	// Software trigger (reset and restart counter)
	PDB0_SC |= PDB_SC_SWTRIG;
	// Enable interrupt request
	NVIC_ENABLE_IRQ(IRQ_PDB);
}


void adc0_isr() {
    adcData = ADC0_RA;
    hasData = true;   
}

void pdb_isr() {
  PDB0_SC &= ~PDB_SC_PDBIF;
}

