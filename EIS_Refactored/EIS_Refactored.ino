#include <ADC.h>
#include <ADC_util.h>
#include "boardnames.h"

#define PI 3.1415926535897932384626433832795

ADC *adc = new ADC(); // adc object;

// ADC::Sync_result sr; //for reading data
// three timers,
// the last for flashing the diode and to show teensy is running
IntervalTimer baseTimer;


/*
  unsigned long st;
  unsigned long  en;
  unsigned long dt;
*/
#define DAC_OUT A21

// Create an array for storing the stimulus
#define BUFFER_SIZE 2000

static volatile uint16_t sinetableL[BUFFER_SIZE];
volatile uint16_t pOut = 0;


float realFreq = 0;
float reqFreq = 50000;
uint32_t freq = 0;
float samplingFreq = 0;

const int readPinD = A10; // ADC0
const int readPinM = A11; // ADC0
const int readPin2 = A16; // ADC1

const int SKIP = 1000;
int skipped = 0;


const int NM = BUFFER_SIZE; //5000 samples

volatile  short int V1[NM]; //loaded by channel 0
volatile unsigned short int V2[NM]; // loaded by channel1
volatile int p1 = 0;
volatile int p2 = 0;


float genAmplitude = 0.2;
int zeroVal = 2000;

float a1 = 0.0;
float a3 = 0.0;
float a5 = 0.0;
boolean DO_FAST = false;


// We need to buffer the serial chars.
const int SERIAL_BUFFER_SIZE = 20;
char commandBuffer[SERIAL_BUFFER_SIZE + 1];
int bufferPos = 0; //position in the buffer
char welcome[] = {"LowCost EIS"};

void clearBuffer() {
  for (int i = 0; i < SERIAL_BUFFER_SIZE + 1; i++)
    commandBuffer[i] = (char)0;
  bufferPos = 0;
}

void printBoardParameters(){
  //identify the used teensy
      Serial.print ("Board: ");
      Serial.print(BOARD);
      Serial.print (", BUS frequency: ");
      Serial.print (F_BUS / 1000000) ;
      Serial.print (", CPU frequency: ");
      Serial.println(F_CPU / 1000000);
      Serial.flush();
}

void setTransientDACValue(){
      commandBuffer[0] = ' ';
      float tmp = atof(commandBuffer);
      
      if (((tmp >= 0) & (tmp <= 4095))) {
        dacEnable();
        analogWrite(DAC_OUT, tmp);
        Serial.print("Transient DAC value requested: ");
        //Serial.println(tmp);
        delay(1000);
        analogWrite(DAC_OUT, 0);
        dacDisable();
      } else {
        Serial.println("Transient DAC value requested is out of range");
      }
      
      Serial.flush();
}

void testTransientDACValue(){
      //test we expect the zero value
      commandBuffer[0] = ' ';
      float tmp = atof(commandBuffer);
      if (((tmp >= 0) & (tmp <= 4095))) {
        dacEnable();
        // analogWrite(DAC_OUT, zeroVal);
        analogWrite(DAC_OUT, tmp);
        //*(volatile int16_t *)&(DAC0_DAT0L) = tmp;
        delay(100);

        adc->adc0->setAveraging(1); // set number of averages
        adc->adc0->setResolution(16); // set bits of resolution
        adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
        adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed
        adc->adc0->disableCompare();
        adc->adc1->setAveraging(1); // set number of averages
        adc->adc1->setResolution(16); // set bits of resolution
        adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
        adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed
        adc->adc1->disableCompare();

        for (int i = 0; i < 10; i++) {
          adc->adc0->analogReadDifferential(readPinD, readPinM);
          adc->adc1->analogRead(readPin2);
        }
        //make some reads
        int x0d = 0;
        unsigned int x1s = 0;
        int NM_ = 500;   //RvE add underscore to variable name, potential conflict with global NM
        for (int i = 0; i < NM_; i++) {
          int x0dv = adc->adc0->analogReadDifferential(readPinD, readPinM);
          unsigned int x1sv = adc->adc1->analogRead(readPin2);

          x0d += x0dv;
          x1s += x1sv;
        }
        x0d /= NM_;

        x1s /= NM_;


        Serial.print("DAC set to:");
        Serial.println(tmp);

        Serial.print("Channel 0 differential:");
        Serial.println(x0d);
        Serial.print("Channel 1 single:");
        Serial.println(x1s);
        Serial.flush();
      } else {
        Serial.println("Out of range");
        Serial.flush();
      }
}

void setProbeFrequency(){
    char buf[100];
    //this is an output frequency request
    commandBuffer[0] = ' ';
    float tmp = atof(commandBuffer);
    reqFreq = tmp;
    sprintf(buf, "Probe frequency set to: %f", reqFreq);
    Serial.println(buf);
}

void setProbeAmplitude(){
    char buf[100];
    //this is an amplitude out change request
    commandBuffer[0] = ' ';
    float tmp = atof(commandBuffer);
    if (((tmp >= 0.01) & (tmp <= 0.6))) {
      genAmplitude = tmp;
      sprintf(buf, "Probe amplitude set to %f", genAmplitude);
      Serial.println(buf);
    } else {
      sprintf(buf, "E05 Probe amplitude out of range, amplitude given: %f, maximum allowed: %f", tmp, 0.6);
      Serial.println(buf);
    }
}

void setProbeDCValue(){
  char buf[100];
  //this is a zero change request
      commandBuffer[0] = ' ';
      float tmp = atof(commandBuffer);
      if (((tmp >= 0) & (tmp <= 4095))) {
        zeroVal = (int)tmp;
        sprintf(buf, "Probe DCValue set to %d", zeroVal);
        Serial.println(buf);
      } else {
        sprintf(buf, "E05 Probe DCValue requested:  %f is out of range maximum value allowed: %f", tmp, 4095.0);
        Serial.println(buf);
      }
}

void setSamplingFrequency(){
  char buf[100];
  //this is an sampling start request
  commandBuffer[0] = ' ';
  float tmp = atof(commandBuffer);
  if ((tmp <= 575000) & (tmp >= 1)) {
    samplingFreq = tmp;
    sprintf(buf, "Sampling frequency for measuremnt set to %f", tmp);
    Serial.println(buf);
  } else {
    sprintf(buf, "E06 requested sampling frequency %f out of range, sampling frequency kept at %f ", tmp, samplingFreq);
    Serial.println(buf);
  }
}

//interrupt functions
void serialEvent() {
  char inChar = Serial.read();
  if (inChar == '\n') {
    //the command is complete what should we do?
    if (commandBuffer[0] == 'Q') {
      testTransientDACValue();
    } else if (commandBuffer[0] == 'F') {
      setProbeFrequency();
    } else if (commandBuffer[0] == 'A') {
      setProbeAmplitude();
    } else if (commandBuffer[0] == 'Y') {
      setProbeDCValue();
    }  else if (commandBuffer[0] == 'G') {
      setSamplingFrequency();
    }  else if (commandBuffer[0] == 'M') {
      startMeasurement(samplingFreq);
    } else if (commandBuffer[0] == 'Z') {
      measureOCP();
    } else if (commandBuffer[0] == 'D' ) {
      sendData();
    } else if (commandBuffer[0] == 'B' ) {
      printBoardParameters();
    } else if (commandBuffer[0] == 'T' ) {
      setTransientDACValue();
    } else {
      //command not recognized 
      Serial.println("DEBUG: Command not recognized: ");
      Serial.println(commandBuffer);
    }
    clearBuffer();
  } else {
    //not a \n
    //if possible queue the chars
    if (bufferPos < BUFFER_SIZE - 1) {
      commandBuffer[bufferPos++] = inChar;
    } else {
      Serial.print("DEBUG: Message too long  ");
      Serial.println(commandBuffer);
      //clear our buffer
      clearBuffer();
      //beware: other characters may arrive
      //and can refill the buffer!
    }
  }
}

void dacDisable() {
  //disable  the DAC output
  Serial.println("DEBUG: DAC Disabled");
  Serial.flush();
  DAC0_C0 &= ~( DAC_C0_DACEN  | DAC_C0_DACRFS);
  pinMode(DAC_OUT, INPUT);
}

void dacEnable() {
  //enable the dac
  Serial.println("DEBUG: DAC Enabled");
  Serial.flush();
  SIM_SCGC2 |= SIM_SCGC2_DAC0;
  DAC0_C0 = DAC_C0_DACEN  | DAC_C0_DACRFS;
  pinMode(DAC_OUT, OUTPUT);
}

void measureOCP() {

  char buf[100];
  //this is an initial OCP measurement request 
  //be sure the output is disconnected ad the DAC turned off
  //disableBothPGA();
  //dacDisable();
  adc->resetError();
  adc->adc1->stopPDB();
  adc->adc1->disableInterrupts();
  //adc->adc1->stopPDB();
  //adc->adc1->disableInterrupts();
  adc->adc1->setReference(ADC_REFERENCE::REF_1V2);
  adc->adc1->setAveraging(32); // set number of averages
  adc->adc1->setResolution(16); // set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED); // change the sampling speed

  int NR = 100;
  uint32_t v1 = 0;
  //int dummy=0;
  for (int i = 0; i < 10; i++) {
    adc->adc1->analogRead(readPin2); // read a new value, will return ADC_ERROR_VALUE if the comparison is false.
  }
  for (int i = 0; i < NR; i++) {
    uint16_t x = adc->adc1->analogRead(readPin2); // read a new value, will return ADC_ERROR_VALUE if the comparison is false.
    v1 += x;
  }
  uint16_t ocpValue = v1 / NR;

  sprintf(buf, "Z%u", ocpValue);
  Serial.println(buf);
}

void startMeasurement(float freq) {
  //clear inputs just in case
  for (int i = 0; i < BUFFER_SIZE; i++) {
    V1[i] = 0;
    V2[i] = 0;
  }

  dacEnable();
  //be sure the adc is set for acquire
  setAdc0ForAcq();
  setAdc1ForAcq();

  skipped = 0;
  p1 = 0;
  p2 = 0;
  pOut = 0;

  realFreq = loadSineTable(reqFreq, freq);
  Serial.print("DEBUG Start pdb with frequency ");
  Serial.print(freq);
  Serial.print(" Hz.");
  Serial.print(" Generated freq ");
  Serial.println(realFreq);
  Serial.flush();
  if (realFreq <= 0.1) {
    adc->adc0->setAveraging(4); // set number of averages
    adc->adc1->setAveraging(4); // set number of averages
    Serial.println("AVG");
    Serial.flush();
  } else {
    adc->adc0->setAveraging(1); // set number of averages
    adc->adc1->setAveraging(1); // set number of averages
    Serial.println("NVG");
    Serial.flush();
  }

  adc->adc0->stopPDB();
  //adc->adc0->startSingleRead(readPinD);// call this to setup everything before the pdb starts, differential is also possible
  adc->adc0->startSingleDifferential(readPinD, readPinM);// call this to setup everything before the pdb starts, differential is also possible
  adc->adc0->enableInterrupts(adc0_isr, 200);
  adc->adc0->startPDB(freq); //frequency in Hz
  adc->adc1->stopPDB();
  adc->adc1->startSingleRead(readPin2); // call this to setup everything before the pdb starts
  adc->adc1->enableInterrupts(adc1_isr, 201);
  adc->adc1->startPDB(freq); //frequency in Hz

}


float loadSineTable(float freq, float samplingFreq) {
  float nPer = -1.0;
  // cleanf the sine table
  for (int x = 0 ; x < BUFFER_SIZE ; x++) {
    sinetableL[x] = 0;
  }


  float dur = BUFFER_SIZE * (1.0 / samplingFreq);

  if (freq < 10000) {
    nPer = (int)(0.5 + dur * freq);
  } else {
    nPer = (int)(0.5 + dur * freq) - 1;
  }
  float realFreq = 1.0 / (dur / nPer);


  boolean sat = false;
  if (DO_FAST)
  {
    Serial.print("DEBUG Fast mode xero is ");
    Serial.println(zeroVal);
    float bit1 = a1 * genAmplitude / 1.2 * 4090;
    float bit3 = a3 * genAmplitude / 1.2 * 4090;
    float bit5 = a5 * genAmplitude / 1.2 * 4090;

    for (int i = 0; i < BUFFER_SIZE; i++) {
      int v = zeroVal + (int)
              (0.5 + bit1 * sin(((float)i) * nPer * 2 * PI / ((float)BUFFER_SIZE)) +
               bit3 * sin(((float)i) * nPer * 6 * PI / ((float)BUFFER_SIZE)) +
               bit5 * sin(((float)i) * nPer * 10 * PI / ((float)BUFFER_SIZE)));
      if (v > 4095) {
        v = 4095;
        sat = true;
      }
      if (v < 0) {
        v = 0;
        sat = true;
      }
      sinetableL[i] = v;
    }
    Serial.print("DEBUG multi dur:");
    Serial.print(dur * 1000);
    Serial.print(" ms");
    Serial.print(" Per :");
    Serial.print(nPer);
    Serial.print(" Freq: ");
    Serial.print(realFreq);
    Serial.print(" Amp: ");
    Serial.print(genAmplitude);
    Serial.print( " (");
    Serial.print(bit1);
    Serial.print(",");
    Serial.print(bit3);
    Serial.print(",");
    Serial.print(bit5);
    Serial.print(")");
    Serial.print(F(" zero "));
    Serial.print(zeroVal);
    if (sat)
      Serial.println(" Saturated");
    else
      Serial.println(" ");

  } else {
    float bitAmplitude = genAmplitude / 1.2 * 4090;
    for (int i = 0; i < BUFFER_SIZE; i++) {
      int v = zeroVal + (int) (0.5 + bitAmplitude * sin(((float)i) * nPer * 2 * PI / ((float)BUFFER_SIZE)));
      if (v > 4095) {
        v = 4095;
        sat = true;
      }
      if (v < 0) {
        v = 0;
        sat = true;
      }
      sinetableL[i] = v;
    }
    Serial.print("DEBUG single dur:");
    Serial.print(dur * 1000);
    Serial.print(" ms");
    Serial.print(" Per :");
    Serial.print(nPer);
    Serial.print(" Freq: ");
    Serial.print(realFreq);
    Serial.print(" Amp: ");
    Serial.print(genAmplitude);
    Serial.print( " (");
    Serial.print(bitAmplitude);
    Serial.print(")");
    Serial.print(F(" zero "));
    Serial.print(zeroVal);
    if (sat)
      Serial.println(" Saturated");
    else
      Serial.println(" ");
  }

  Serial.flush();
  return realFreq;
}




void baseIrq() {
  //this should run always
  digitalWriteFast(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

// Make sure to call readSingle() to clear the interrupt. FASTRUN
void adc0_isr() {
  //st=micros();
  pOut++;
  pOut = pOut % BUFFER_SIZE;
  
  
  analogWrite(DAC_OUT, sinetableL[pOut]);
  // *(volatile int16_t *)&(DAC0_DAT0L) = sinetableL[pOut];
  
  if (p1 >= NM ) {
    adc->adc0->readSingle();
    adc->adc0->stopPDB();
    dacDisable();
    // Serial.print("ERROR Trying to read beyond end of buffer. ");
    Serial.print("DONE: 0");
    Serial.print(", NM: ");
    Serial.print(NM);
    Serial.print(", p1: ");
    Serial.print(p1);
    Serial.flush();
    return;
  }
  
  if (skipped < SKIP) {
    skipped ++;
    adc->adc0->readSingle();
    return;

  }
  V1[p1++] = adc->adc0->readSingle(); // read a new value, will return ADC_ERROR_VALUE if the comparison is false.
  //en=micros();
  //dt=en-st;
}

void adc1_isr() {
  if (p2 >= NM ) {
    adc->adc1->readSingle();
    adc->adc1->stopPDB();
    dacDisable();
    Serial.println("DONE: 1");
    Serial.flush();
    return;
  }
  if (skipped < SKIP) {
    adc->adc1->readSingle();
    return;

  }
  V2[p2++] = (uint16_t)adc->adc1->readSingle();

}

void setAdc0ForAcq() {
  adc->adc0->setAveraging(1); // set number of averages
  adc->adc0->setResolution(16); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed
  adc->adc0->disableCompare();
  //adc->adc0->enablePGA(pga1Gain);  // gain can be 1,2,4,8,16,32,64
}
void setAdc1ForAcq() {
  adc->adc1->setAveraging(1); // set number of averages
  adc->adc1->setResolution(16); // set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed
  adc->adc1->disableCompare();
  //adc->adc1->enablePGA(pga1Gain);  // gain can be 1,2,4,8,16,32,64
}


void sendData() {

  //this is an sampling start request
  commandBuffer[0] = ' ';
  int tmp = atoi(commandBuffer);

  if (tmp == 0){
    Serial.print("DEBUG size:");
    Serial.print(BUFFER_SIZE);
    Serial.print(" Gen freq:");
    Serial.print(realFreq);
    Serial.print(" Sampling freq:");
    Serial.println(samplingFreq);
  
    Serial.print("#DATA,");
    //send the number of data
    Serial.print(BUFFER_SIZE);
    Serial.print(",");
    //send the frequency
    Serial.print(realFreq);
    Serial.print(",");
    //send the acquisition freq
    Serial.println(samplingFreq);
    //Serial.print(",");
    Serial.flush();
    } else if (tmp == 1){
      //send all data
      for (int i = 0; i < BUFFER_SIZE; i++) {
        Serial.print(V1[i]);
        Serial.print(",");
      }
    } else if (tmp == 2){
      //send all data
      for (int i = 0; i < BUFFER_SIZE; i++) {
        Serial.print(V2[i]);
        Serial.print(",");
        }
    } else if (tmp == 3){
      //send all data
      for (int i = 0; i < BUFFER_SIZE; i++) {
        Serial.print(sinetableL[i]);
        Serial.print(",");
        }
    }else {
      Serial.print("Future extension aka nNot Implemented");
    }
    
    
  //tag the end of sending
  Serial.print("#END");
  Serial.flush();
}


void setup() {

  Serial.begin(115000);
  delay(400);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(readPinD, INPUT);
  pinMode(readPinM, INPUT);
  pinMode(readPin2, INPUT);
  Serial.println("DEBUG Begin setup");


  Serial.flush();
  SIM_SCGC2 |= SIM_SCGC2_DAC0;
  DAC0_C0 = DAC_C0_DACEN;  // 1.2V ref is DACREF_1
  analogReference(0);  // 1: Vref set to 1.2V, 0: Vref set to 3.3 V
  //setAdc0ForAcq();
  //setAdc1ForAcq();
  adc->resetError();
  analogWriteResolution(12);   //values between 0 and 4095
  analogWrite(DAC_OUT, 1000);
  dacDisable();
  /*
    Serial.print("C0:");
    Serial.print(DAC0_C0, HEX);
    Serial.print(" C1:");
    Serial.print(DAC0_C1, HEX);
    Serial.print(" C2:");
    Serial.println(DAC0_C2, HEX);
  */
  adc->resetError();

  //start the continuous timer
  baseTimer.priority(180);
  baseTimer.begin(baseIrq, 1000000);

  Serial.println("DEBUG End setup");
  Serial.flush();

}

void loop() {
  delay(100);
}
