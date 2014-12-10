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

#define NZEROS 8
#define NPOLES 8
#define GAIN   4.549356222e+01

//------------------------- Glabals -----------------------------------------

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
uint16_t lineDelta;
uint16_t xPos = 0;
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
int i2 = 0;
uint32_t startTime;
volatile boolean to_munu_state;
volatile int prev2;

//----------------------------------- Queue Methods ------------------------------------

typedef struct Queue
{
  uint32_t * vals;
  uint32_t startPoint;
  uint32_t endPoint;
  uint32_t length;
  uint32_t queueSize;
} Queue;


uint32_t peekEndQueue(Queue * q) { 
  int index = q->endPoint - 1;
  index = (index >= 0) ? index : index + q->length;
  return q->vals[index];
}


uint32_t peekQueue(Queue * q) {
  return q->vals[q->startPoint];
}

uint32_t removeQueue(Queue * q) {
    q->queueSize--;
    uint32_t temp = q->vals[q->startPoint];
    q->startPoint++;
    q->startPoint = q->startPoint % q->length;
    return temp;
}
 
void addQueue(Queue * q, uint32_t value) {
  q->queueSize++;
  q->vals[q->endPoint] = value;
  q->endPoint++;
  q->endPoint = q->endPoint % q->length;
}

void resetQueue(Queue * q) {
  q->startPoint = 0;
  q->endPoint = 0;
  q->queueSize = 0;
}

Queue * initQueue(uint32_t queueLength) {
  Queue * ret = (Queue *) malloc(sizeof(Queue));
  ret->vals = (uint32_t *) malloc(sizeof(uint32_t) * queueLength);
  ret->length = queueLength;
  return ret;
}

//------------------------------ File Methods -----------------------------

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

void checkReadBack() {
  int sensorVal = digitalRead(0);

  if (prev2 != sensorVal && sensorVal == LOW) {
    readFile(sampleNumber);
  }
  prev2 = sensorVal;

}
  
//-------------------------------- Init Methods ---------------------------------------------

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
    resetQueue();
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

void initHorLines() {
  for(int i = 0; i <= tft.height() / grid_delta; i++) {
    tft.drawLine(0, i * grid_delta, tft.width(), i * grid_delta, GRAPH_GRID_LINES);
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


//---------------------- Grid Drawing Util -------------------------------
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

//----------------------- Filtering -----------------------------------------


static float xv[NZEROS+1], yv[NPOLES+1];

static float filterloop(float input)
  { for (;;)
      { xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; xv[4] = xv[5]; xv[5] = xv[6]; xv[6] = xv[7]; xv[7] = xv[8]; 
        xv[8] =  input  / GAIN;
        yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; yv[4] = yv[5]; yv[5] = yv[6]; yv[6] = yv[7]; yv[7] = yv[8]; 
        yv[8] =   (xv[0] + xv[8]) - 4 * (xv[2] + xv[6]) + 6 * xv[4]
                     + ( -0.0656274072 * yv[0]) + (  0.6803537539 * yv[1])
                     + ( -3.1973474110 * yv[2]) + (  8.7164421149 * yv[3])
                     + (-15.1907165280 * yv[4]) + ( 17.4196484110 * yv[5])
                     + (-12.7689074570 * yv[6]) + (  5.4061545144 * yv[7]);
        return yv[8];
      }
  }


uint8_t numValues = 2;
volatile uint32_t runningAverage = 0;

float transform(uint32_t input) {
    runningAverage = (runningAverage * numValues - runningAverage + (filterloop(((float) input) - 2048) + 2048)) / numValues;
  return runningAverage;  
}

float averageSlope(uint16_t numToLookBack) {
  int32_t tempEnd = sdIndex - 1;
  float ret = 0.0;
  for (int i = 0; i < numToLookBack; i++) {
    if (tempEnd <= 0) {
      numToLookBack = i;
    } else {
      ret += (sdOutput[tempEnd] - sdOutput[tempEnd - 1]) * (sdOutput[tempEnd] - sdOutput[tempEnd - 1]);
    }
    tempEnd--;
  }
  numToLookBack = (numToLookBack == 0) ? 1 : numToLookBack;
  return ret / (numToLookBack);
}

float integrate(float qrs[], int leng) {
  float ret = 0.0;
  for (int i = 0; i < leng; i++) {
    ret += qrs[i];
  }
  return ret;
}

//--------------------------------- Main ---------------------------------------------------------

void loop() {
  uint32_t time = millis();
  
  //check if the button is pressed
  int sensorVal = digitalRead(1);
  if (prev != sensorVal && sensorVal == LOW) {
    if(menu_state) {
      initReadingDataState(time);
    } else if(reading_state) {
      to_menu_state = true;    
    }
  }
  prev = sensorVal;
 
 
 //timeout   
 if(to_menu_state || reading_state && time - count > 300000 || sdIndex == HERTZ * RECORD_TIME) {
   writeToSD(); 
   sampleNumber++;
   sampleNumber = sampleNumber % 100;
   sdIndex = 0;
   initMenuState();
 } 

//do we have data and am in a reading state
if (hasData && reading_state) {
    hasData = false;    
    float dataPoint = transform(adcData);
    uint32_t yVal = tft.height() *  dataPoint / 4095;
    
    //if the queue is full remove a line
    if (queueSize == QUEUE_LENGTH) {
      removeLine();
    } 
    
    //grab the oldVal and add a line to the graph
    uint32_t oldYVal = peekEndQueue();
    addLine(oldYVal, yVal);
    
    //add the YVal to the queue and data to the sd output
    addQueue(yVal);
    sdOutput[sdIndex] = dataPoint;
    sdIndex++;  
  }   
}

void adc0_isr() {
    adcData = ADC0_RA;
    hasData = true;   
}

void pdb_isr() {
  PDB0_SC &= ~PDB_SC_PDBIF;
}

//-------------------------------------- Helpers -------------------------------------------

void removeLine() {
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

void addLine(uint32_t temp, uint32_t yVal) {
  tft.drawLine(xPos, temp, xPos + lineDelta, yVal, GRAPH_LINE);
  tft.drawLine(xPos, temp - 1, xPos + lineDelta, yVal - 1, GRAPH_LINE);
  tft.drawLine(xPos, temp + 1, xPos + lineDelta, yVal + 1, GRAPH_LINE);
  xPos += lineDelta;
  if (xPos > tft.width()) {
    xPos = 0;        
       
  }
}





