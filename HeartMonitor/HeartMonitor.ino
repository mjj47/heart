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
#define BOTTOM_BOX_COLOR ILI9341_BLACK
#define BOTTOM_TEXT_COLOR ILI9341_WHITE
#define HEART_BEAT_COLOR ILI9341_RED
#define ROUND_RECT_COLOR ILI9341_RED

#define NZEROS 8
#define NPOLES 8
#define GAIN   4.549356222e+01


const uint8_t chipSelect = SD_CS;
SdFat sd;
File file;
char fileName[13] = FILE_BASE_NAME "00.CSV";
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
uint32_t PDB_CONFIG;

//TODO: move to defines
const int HERTZ = 250;
const float TIME_GRID_DELTA = .04;
const int HEART_INPUT = 14;
const int DISPLAY_QUEUE_LENGTH = 321;


//Graph Vars
uint32_t num_vert_lines = (float) DISPLAY_QUEUE_LENGTH / HERTZ / TIME_GRID_DELTA;
uint32_t grid_delta; 
uint32_t bottom_box_y;
uint16_t lineDelta;
uint16_t xPos = 0;
uint32_t oldXpos = 0;
uint16_t blankSpace = 0;

//Button globals
int prevUp = HIGH;
int prevSelect = HIGH;
int prevDown = HIGH;
bool upEvent = false;
bool selectEvent = false;
bool downEvent = false;

//state and state transitions 
bool readState = false;
bool menuState = false;
bool fileChooseState = false;
bool reportState = false;
bool recallSate = false;

bool toMenuState = false;
bool toReadState =  false;
bool toFileChooseState = false;
bool toReportState = false;
bool toRecallState = false;

bool clearRed = false;
bool hasBPM = false;
bool hasData = false;


//TODO: temperal and frame rate vars
uint32_t qrsDetected = 0;
uint32_t readStateStartTime = 0;


volatile uint32_t adcData;

//SD card Vars
//move to define
const uint8_t RECORD_TIME = 30;

uint32_t sdOutput[HERTZ * RECORD_TIME];
uint32_t sdIndex = 0;
uint16_t sampleNumber = 0;

//menu state vars
int menuChoose = 0;

//recall state vars
int sdRecallIndex = 0;


//-------------------------------------------------------------------------------------
//----------------------------------- Queue Methods ------------------------------------
//-------------------------------------------------------------------------------------



typedef struct queue_struct
{
  uint32_t* vals;
  uint32_t startPoint;
  uint32_t endPoint;
  uint32_t max_length;
  uint32_t queueSize;
} Queue;

const int QRS_QUEUE_LENGTH = 4;
const int QRS_BPM_LENGTH = 4;

Queue* graphDisplay;
Queue* qrs;
Queue* qrsTimes;

uint32_t peekEndQueue(void * t) {
  Queue * q = (Queue *) t; 
  int index = q->endPoint - 1;
  index = (index >= 0) ? index : index + q->max_length;
  return q->vals[index];
}


uint32_t peekQueue(void* t) {
  Queue * q = (Queue *) t;
  return q->vals[q->startPoint];
}

uint32_t removeQueue(void* t) {
  Queue * q = (Queue *) t;
  q->queueSize--;
  uint32_t temp = q->vals[q->startPoint];
  q->startPoint++;
  q->startPoint = q->startPoint % q->max_length;
  return temp;
}
 
void addQueue(void* t, uint32_t value) {
  Queue * q = (Queue *) t;
  q->queueSize++;
  q->vals[q->endPoint] = value;
  q->endPoint++;
  q->endPoint = q->endPoint % q->max_length;
}

void resetQueue(void* t) {
  Queue * q = (Queue *) t;
  q->startPoint = 0;
  q->endPoint = 0;
  q->queueSize = 0;
}

void* initQueue(uint32_t queueLength) {
  Queue* ret = (Queue*) malloc(sizeof(Queue));
  ret->vals = (uint32_t *) malloc(sizeof(uint32_t) * queueLength);
  ret->max_length = queueLength;
  return ret;
}

//-------------------------------------------------------------------------------------
//-------------------------------------- File Methods ---------------------------------
//-------------------------------------------------------------------------------------

int16_t fileNameIndex = 0;
char* fileNames[100];
uint16_t maxFileIndex = 0;

void readFileNames() {
  int i;
  maxFileIndex = 0;
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    Serial.println("Can't write to SD card");
  }
  for(i = 0; file.openNext(sd.vwd(), O_READ); i++) {
    char* name = file.name();
    if (name[0] != 'D' || name[1] != 'A' || name[2] != 'T' || name[3] != 'A') {
      file.close();
      continue;
    }
    for (int j = 0; j < 10; j++) {
      fileNames[maxFileIndex][j] = name[j];
    }
    maxFileIndex++;
    file.close();
  }
}


void moveNextFile() {
  if (fileNameIndex + 1 != maxFileIndex){
    fileNameIndex++;
  }
}

void movePrevFile() {
  if (fileNameIndex != 0) {
    fileNameIndex--;
  }
}

char * getCurrentFile() {
  return fileNames[fileNameIndex];
}


void setFileName(uint16_t index) {
  int nums = 4;
  
  fileName[nums] = '0' + ((index / 10) % 10);
  fileName[nums + 1] = '0' + (index % 10);
}

void readFile() {
  char* name = (char *) getCurrentFile(); 
  fileName[4] = name[4];
  fileName[5] = name[5]; 
  file.open(fileName);
  uint32_t val = 0;
  uint32_t indexInFile = 0;
  sdIndex = 0;
  int c;
  while ((c = file.read()) >= 0) {
    Serial.println(c);
    if (c >= '0' && c <= '9') {
      val = val * 10 + (uint32_t) (c - '0');
    } else if ((indexInFile > 12 && c == ',') ||(indexInFile > 12 && c == '\n' )) {
      sdOutput[sdIndex] = val;
      val = 0;
      sdIndex++;
    }
    indexInFile++;;  
  }
  sdOutput[sdIndex] = val;
  sdIndex++;
  Serial.println(sdIndex);
  file.close();
  

}

void openFile(uint16_t index) {
  setFileName(index);
  Serial.println("Attempting to Open File... "); 
  if (!file.open(fileName, FILE_WRITE)) {
    //error("file.open");
    Serial.print("Could Not open File: ");
  } else {
    Serial.print("Opened File: ");
  }
  Serial.println(fileName);
}

void writeToSD() { 
  setFileName(sampleNumber);
  sd.remove(fileName);
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

//--------------------------------------------------------------------------------------------------------  
//----------------------------------- Setup and Init Methods ---------------------------------------------
//--------------------------------------------------------------------------------------------------------  


void setup() {
  tft.begin();
  tft.setRotation(1);
  pinMode(2, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(0, INPUT_PULLUP);
  pinMode(HEART_INPUT, INPUT);

  Serial.begin(9600);
  while (!Serial);

  graphDisplay = (Queue *) initQueue(DISPLAY_QUEUE_LENGTH);
  qrs = (Queue *) initQueue(QRS_QUEUE_LENGTH);
  qrsTimes = (Queue *) initQueue(QRS_BPM_LENGTH);
  
  grid_delta = tft.width() / num_vert_lines;
  lineDelta = tft.width() / (DISPLAY_QUEUE_LENGTH - 1);
  
  Serial.println("woot");
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    Serial.println("Can't write to SD card");
  } else {
    Serial.println("SD card set up");
    for (int i = 0; i < 100; i++) {
      fileNames[i] = (char*) malloc(sizeof(char) * 11);
      fileNames[i][10] = '\0';
    }
  }
  
  adcInit();
  pdbInit();
  
  NVIC_ENABLE_IRQ(IRQ_PDB);
  initMenuState();
  menuState = true;
}


//---------------- Init Menu State  ---------------------------

void initMenuState() {
  tft.fillScreen(MENU_BACKGROUND);
  tft.setTextColor(MENU_HEADER);
  tft.setTextSize(3);
  tft.setCursor(65, 10); tft.print("EKG Monitor");
  tft.setTextSize(2);
  tft.setTextColor(MENU_TEXT);

  tft.setCursor(80, 100); tft.print("Start Reading"); 
  tft.setCursor(95, 165); tft.print("Recall Data");  

  if(menuChoose == 0) {
    tft.drawRoundRect(60, 83, 200, 50, 10,  ROUND_RECT_COLOR);
  } else {
    tft.drawRoundRect(60, 148, 200, 50, 10,  ROUND_RECT_COLOR); 
  }
}

//---------------- Init File Choose State --------------------

void initFileChooseState() {
  tft.fillScreen(MENU_BACKGROUND);
  tft.setTextColor(MENU_HEADER);
  tft.setTextSize(3);
  tft.setCursor(65, 10); tft.print("Choose File");
  readFileNames();   
  tft.setTextSize(2); 
  tft.setTextColor(MENU_TEXT);
  char * name = (char *) getCurrentFile();
  tft.setCursor(110,120); tft.print(name);
  drawDownArrow();

}

//------------------ Init Recall State --------------------------

void initRecallState() {
  sdRecallIndex = 0;
  readFile();
  drawGraphSection(-1, 0);
}

//------------------- Init Read State ------------------------

void initReadDataState(uint32_t time) {
    tft.fillScreen(GRAPH_BACKGROUND);
    tft.fillRect(0, bottom_box_y, tft.width(), bottom_box_y, BOTTOM_BOX_COLOR);   
    tft.setCursor(tft.width() / 2 - 80, tft.height() / 2);
    tft.setTextColor(CALIBRATING);
    tft.setTextSize(2);
    tft.print("Callibrating....");
    bottom_box_y = tft.height() - 30;
    int maxV = -1;
    int minV = 1024;
    int index = 0;
    int calBuf[300];
    int calBufSize = 0;
    while(abs(maxV - minV) > 1100) {
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
    resetQueue(graphDisplay);
    resetQueue(qrs);

    xPos = 0;
    
    //reset the sd and time
    readStateStartTime = time;
    sdIndex = 0;
    
    //add the first filler data point
    addQueue(graphDisplay, 0);
}

//--------------------- Init Report State -------------------------
void initReportState() {
  tft.fillScreen(MENU_BACKGROUND);
  tft.setTextColor(MENU_HEADER);
  tft.setTextSize(3);
  tft.setCursor(45, 10); tft.print("Report State");
}


//--------------------------- Init Helper methods ------------



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



//-------------------------------------------------------------------------------------------
//--------------------------------- Filtering -----------------------------------------------
//-------------------------------------------------------------------------------------------


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


//----------------------------------------------------------------------------------------------------------
//------------------------------------------- LOOP ---------------------------------------------------------
//----------------------------------------------------------------------------------------------------------


void loop() {
  uint32_t time = millis();
  int sensorVal;
  //check if the down button was pressed
  sensorVal = digitalRead(0);
  if (prevDown != sensorVal && sensorVal == LOW) {
    downEvent = true;
  }
  prevDown = sensorVal;

  //check if the select button was pressed
  sensorVal = digitalRead(1);
  if (prevSelect != sensorVal && sensorVal == LOW) {
    selectEvent = true;
  }
  prevSelect = sensorVal;

  //check if the up button was pressed
  sensorVal = digitalRead(2);
  if (prevUp != sensorVal && sensorVal == LOW) {
    upEvent = true;
  }
  prevUp = sensorVal;

  //-------------------------- Menu State Actions ---------------------------------
  if(menuState) {
    //select event

    if(selectEvent) {
      //determine what text is picked
      if(menuChoose == 0) {
        toReadState = true;
      } else {
        toFileChooseState = true;
      }     
    }

    //down or up event
    if(downEvent || upEvent) {
      if(menuChoose == 0 && downEvent) {
        menuChoose = 1;
      }
      if(menuChoose == 1 && upEvent) {
        menuChoose = 0;
      }
      if(menuChoose == 0) {
        tft.drawRoundRect(60, 83, 200, 50, 10,  ROUND_RECT_COLOR);
        tft.drawRoundRect(60, 148, 200, 50, 10,  MENU_BACKGROUND); 
      } else {
        tft.drawRoundRect(60, 83, 200, 50, 10,  MENU_BACKGROUND);
        tft.drawRoundRect(60, 148, 200, 50, 10,  ROUND_RECT_COLOR); 
      }
    }
  }

  //-------------------------- File Choose State Actions -------------------------------
  if(fileChooseState) {
    if(selectEvent) {
      toRecallState = true;
    } else if (downEvent) {
      tft.fillRect(0,40,tft.width(), tft.height() - 40, MENU_BACKGROUND);
      moveNextFile();
      char * name = getCurrentFile();
      drawUpArrow();
      if (fileNameIndex + 1 != maxFileIndex) {drawDownArrow();}
      tft.setCursor(110,120); tft.print(name);
    } else if (upEvent) {
      tft.fillRect(0,40,tft.width(), tft.height() - 40, MENU_BACKGROUND);
      movePrevFile();
      char * name = getCurrentFile();
      drawDownArrow();
      if (fileNameIndex != 0) {drawUpArrow();}
      tft.setCursor(110,120); tft.print(name);

    }
  }


  //-------------------------- Read State Actions -------------------------------
  if(readState) {
    if(selectEvent || sdIndex == HERTZ * RECORD_TIME - 1 || time - readStateStartTime > 300000) {
      toReportState = true;
    }
    if(hasData) {
      hasDataAction(time);
    }
    if(hasBPM) {
      hasBPMAction();
    }
    if(time - qrsDetected > 100 && clearRed) {
      clearRedAction();
    }
  }

  //-------------------------- Report State Actions -------------------------------
  if(reportState) {
    if(selectEvent) {
      toMenuState = true;
    }
  }

  //-------------------------- Recall State Actions -------------------------------
  if(recallSate) {
    if(selectEvent) {
      toMenuState = true;
    } 
    if(upEvent || downEvent) {
      uint32_t oldIndex = sdRecallIndex;
      if(upEvent) {
        sdRecallIndex = min(HERTZ * RECORD_TIME - DISPLAY_QUEUE_LENGTH, sdRecallIndex +  DISPLAY_QUEUE_LENGTH / 2);
      }
      if(downEvent) {
        sdRecallIndex = max(0, sdRecallIndex - DISPLAY_QUEUE_LENGTH / 2);
      }
      drawGraphSection(oldIndex, sdRecallIndex);
    }
  }



  //----------------------------- State Transitons ------------------------------------
  if(toMenuState) {
    initMenuState();
    menuState = true;
    reportState = false;
    recallSate = false;
    toMenuState = false;
  }

  if(toReportState) {
    writeToSD(); 
    sampleNumber++;
    sampleNumber = sampleNumber % 100;
    sdIndex = 0;
    reportState = true;
    readState = false;
    toReportState = false;
    initReportState();
  }

  if(toReadState) {
    menuState = false;
    readState = true;
    toReadState = false;
    initReadDataState(time);
  }

  if(toFileChooseState) {
    toFileChooseState = false;
    fileChooseState = true;
    menuState = false;
    initFileChooseState();
  }

  if(toRecallState) {
    recallSate = true;
    toRecallState = false;
    fileChooseState = false;
    initRecallState();
  }

  //clear the selection events
  upEvent = false;
  downEvent = false;
  selectEvent = false;
}



void adc0_isr() {
    adcData = ADC0_RA;
    hasData = true;   
}

void pdb_isr() {
  PDB0_SC &= ~PDB_SC_PDBIF;
}

//-------------------------------------- Helpers Methopds -------------------------------------------

void removeLine() {
  uint32_t oldVal = removeQueue(graphDisplay);
  uint32_t oldVal2 = peekQueue(graphDisplay);
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

void drawUpArrow() {
  tft.drawLine(160, 100, 160, 65, MENU_TEXT);
  tft.drawLine(160, 65, 150, 75, MENU_TEXT);
  tft.drawLine(160, 65, 170, 75, MENU_TEXT);

}

void drawDownArrow() {
  tft.drawLine(160, 150, 160, 185, MENU_TEXT);
  tft.drawLine(160, 185, 150, 175, MENU_TEXT);
  tft.drawLine(160, 185, 170, 175, MENU_TEXT);
}

void hasDataAction(uint32_t time) {
  float dataPoint = transform(adcData);
  hasData = false;    
  uint32_t yVal = min(tft.height() - tft.height() *  dataPoint / 4095, bottom_box_y - 1) ;
  
  //if the queue is full remove a line
  if (graphDisplay->queueSize == DISPLAY_QUEUE_LENGTH) {
    removeLine();
  } 
  
  //grab the oldVal and add a line to the graph
  uint32_t oldYVal = peekEndQueue(graphDisplay);
  addLine(oldYVal, yVal);
  
  //add the YVal to the queue and data to the sd output
  addQueue(graphDisplay, yVal);
  sdOutput[sdIndex] = dataPoint;
  sdIndex++;  

  //qrs detect
  uint32_t slope = averageSlope(10);
  if (slope > 9000 && time - qrsDetected > 200) {

    hasBPM = true;
    addQueue(qrs, time);
    qrsDetected = time;

    //draw the debugging line
    tft.drawLine(oldXpos, 0, oldXpos, 10, GRAPH_BACKGROUND);
    tft.drawLine(xPos, 0, xPos, 10, GRAPH_LINE);
    oldXpos = xPos;    
  }
}

void hasBPMAction() {
  uint16_t bpm = getBPM();
  tft.fillRect(0, bottom_box_y, tft.width() / 2, tft.height() - bottom_box_y, BOTTOM_BOX_COLOR);
  tft.setTextColor(BOTTOM_TEXT_COLOR);
  tft.setTextSize(2);
  tft.setCursor(0, tft.height() - 25); tft.print("BPM: "); tft.print(bpm);
  tft.fillRect(tft.width() / 2, bottom_box_y, tft.width(), tft.height() - bottom_box_y, HEART_BEAT_COLOR);
  hasBPM = false;
  clearRed = true;
}

void clearRedAction() {
  tft.fillRect(tft.width() / 2, bottom_box_y, tft.width(), tft.height() - bottom_box_y, BOTTOM_BOX_COLOR);
  clearRed = false;
}

uint32_t getBPM() {
 int bpmIndex = qrs->endPoint;
 double diff = 0.0;
 
 for (int i = 0; i < qrs->max_length - 1; i++) {

    diff += qrs->vals[(bpmIndex + i + 1) % qrs->max_length] - qrs->vals[(bpmIndex + i) % qrs->max_length];
 }
 diff = diff / (qrs->max_length - 1);
 diff = 60000 / diff;
 return diff;
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

//Used when recalling data from sd card
void drawGraphSection(uint32_t oldIndex, uint32_t newIndex) {
  if(oldIndex != -1) {
    uint32_t tempXpos = 0;
    uint32_t oldyVal = tft.height() - tft.height() *  sdOutput[sdRecallIndex] / 4095;
    for(int i = oldIndex + 1; i < oldIndex + DISPLAY_QUEUE_LENGTH; i++) {
      uint32_t yVal = tft.height() - tft.height() *  sdOutput[i] / 4095;
      tft.drawLine(tempXpos, oldyVal, tempXpos + lineDelta,
                  yVal, GRAPH_BACKGROUND);
      tft.drawLine(tempXpos, oldyVal + 1, tempXpos + lineDelta,
                  yVal + 1, GRAPH_BACKGROUND);                
      tft.drawLine(tempXpos, oldyVal - 1, tempXpos + lineDelta,
                  yVal - 1, GRAPH_BACKGROUND);
      redrawVertLines(tempXpos, oldyVal, tempXpos + lineDelta, yVal);
      redrawHorLines(tempXpos, oldyVal, tempXpos + lineDelta, yVal);

      oldyVal = yVal;
      tempXpos += lineDelta;
    }
  }

  uint32_t tempXpos = 0;
  uint32_t oldyVal = tft.height() - tft.height() *  sdOutput[sdRecallIndex] / 4095;
  for(int i = newIndex + 1; i < newIndex + DISPLAY_QUEUE_LENGTH; i++) {
    uint32_t yVal = tft.height() - tft.height() *  sdOutput[i] / 4095;
    tft.drawLine(tempXpos, oldyVal, tempXpos + lineDelta, yVal, GRAPH_LINE);
    tft.drawLine(tempXpos, oldyVal - 1, tempXpos + lineDelta, yVal - 1, GRAPH_LINE);
    tft.drawLine(tempXpos, oldyVal + 1, tempXpos + lineDelta, yVal + 1, GRAPH_LINE);
    oldyVal = yVal;
    tempXpos += lineDelta;
  }
}









