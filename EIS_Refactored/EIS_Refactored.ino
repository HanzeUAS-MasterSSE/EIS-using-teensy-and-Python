// The code in this sketch is based on code provided by the authors of the paper:
//   'Electrochemical Impedance Spectroscopy System Based on a Teensy Board', 
//   Leila Es Sebar, Leonardo Iannucci, Emma Angelini, Sabrina Grassini, and Marco Parvis,
//   IEEE TRANSACTIONS ON INSTRUMENTATION AND MEASUREMENT, VOL. 70, 2021
// 
// It was refactored and rewritten for use with the Python serial module by:
//      Ronald van Elburg (www.vanelburg.eu)

#include <ADC.h>
#include <ADC_util.h>
#include "boardnames.h"

#define PI 3.1415926535897932384626433832795

// Create a timer for flashing the diode and to show teensy is running
IntervalTimer baseTimer;


// Create an ADC object for managing the analog to digital converter.
// The analog to digital converter is used to measure the system response,
// while the digital to analog converter is used to define the stimulus.
ADC *adc = new ADC(); // adc object;


// Give the pins in use a name
const int pin_writeDAC = A21;   
const int pin_readD = A10; // ADC0
const int pin_readM = A11; // ADC0
const int pin_read2 = A16; // ADC1


// Create an array for storing the stimulus
const int stimulus_length = 2000;
static volatile uint16_t stimulus_table[stimulus_length];
volatile uint16_t pOut = 0;   //  position in output array


// Define stimulus parameters 
float f_requested = 50000;   // stimulus frequency
float A_stimulus = 0.2;     // stimulus amplitude
int DC_offset_stimulus = 2000;               // stimulus DC offset

float f_realized = 0;
float f_sampling = 0;

const int SKIP = 1000;
int skipped = 0;


// Allocate memory for storing the measured response
volatile short int V1[stimulus_length]; //loaded by channel 0
volatile unsigned short int V2[stimulus_length]; // loaded by channel1

volatile int p1 = 0;
volatile int p2 = 0;


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


void setTransientDACValue(){
      commandBuffer[0] = ' ';
      float tmp = atof(commandBuffer);
      
      if (((tmp >= 0) & (tmp <= 4095))) {
        dacEnable();
        analogWrite(pin_writeDAC, tmp);
        Serial.print("Transient DAC value requested: ");
        //Serial.println(tmp);
        delay(1000);
        analogWrite(pin_writeDAC, 0);
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
        // analogWrite(pin_writeDAC, DC_offset_stimulus);
        analogWrite(pin_writeDAC, tmp);
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
          adc->adc0->analogReadDifferential(pin_readD, pin_readM);
          adc->adc1->analogRead(pin_read2);
        }
        //make some reads
        int x0d = 0;
        unsigned int x1s = 0;
        int NM_ = 500;   //RvE add underscore to variable name, potential conflict with global NM
        for (int i = 0; i < NM_; i++) {
          int x0dv = adc->adc0->analogReadDifferential(pin_readD, pin_readM);
          unsigned int x1sv = adc->adc1->analogRead(pin_read2);

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
    f_requested = tmp;
    sprintf(buf, "Probe frequency set to: %f", f_requested);
    Serial.println(buf);
}

void setProbeAmplitude(){
    char buf[100];
    //this is an amplitude out change request
    commandBuffer[0] = ' ';
    float tmp = atof(commandBuffer);
    if (((tmp >= 0.01) & (tmp <= 0.6))) {
      A_stimulus = tmp;
      sprintf(buf, "Probe amplitude set to %f", A_stimulus);
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
        DC_offset_stimulus = (int)tmp;
        sprintf(buf, "Probe DCValue set to %d", DC_offset_stimulus);
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
    f_sampling = tmp;
    sprintf(buf, "Sampling frequency for measuremnt set to %f", tmp);
    Serial.println(buf);
  } else {
    sprintf(buf, "E06 requested sampling frequency %f out of range, sampling frequency kept at %f ", tmp, f_sampling);
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
      startMeasurement(f_sampling);
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
    if (bufferPos < stimulus_length - 1) {
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
  pinMode(pin_writeDAC, INPUT);
}

void dacEnable() {
  //enable the dac
  Serial.println("DEBUG: DAC Enabled");
  Serial.flush();
  SIM_SCGC2 |= SIM_SCGC2_DAC0;
  DAC0_C0 = DAC_C0_DACEN  | DAC_C0_DACRFS;
  pinMode(pin_writeDAC, OUTPUT);
}

void measureOCP() {

  char buf[100];
  //this is an initial OCP measurement request 
  //be sure the output is disconnected and the DAC turned off
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
    adc->adc1->analogRead(pin_read2); // read a new value, will return ADC_ERROR_VALUE if the comparison is false.
  }
  for (int i = 0; i < NR; i++) {
    uint16_t x = adc->adc1->analogRead(pin_read2); // read a new value, will return ADC_ERROR_VALUE if the comparison is false.
    v1 += x;
  }
  uint16_t ocpValue = v1 / NR;

  sprintf(buf, "Z%u", ocpValue);
  Serial.println(buf);
}

void startMeasurement(float freq) {
  //clear inputs just in case
  for (int i = 0; i < stimulus_length; i++) {
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

  f_realized = loadSineTable(f_requested, freq);
  Serial.print("DEBUG Start pdb with frequency ");
  Serial.print(freq);
  Serial.print(" Hz.");
  Serial.print(" Generated freq ");
  Serial.println(f_realized);
  Serial.flush();
  if (f_realized <= 0.1) {
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
  //adc->adc0->startSingleRead(pin_readD);// call this to setup everything before the pdb starts, differential is also possible
  adc->adc0->startSingleDifferential(pin_readD, pin_readM);// call this to setup everything before the pdb starts, differential is also possible
  adc->adc0->enableInterrupts(adc0_isr, 200);
  adc->adc0->startPDB(freq); //frequency in Hz
  adc->adc1->stopPDB();
  adc->adc1->startSingleRead(pin_read2); // call this to setup everything before the pdb starts
  adc->adc1->enableInterrupts(adc1_isr, 201);
  adc->adc1->startPDB(freq); //frequency in Hz

}


float loadSineTable(float freq, float f_sampling) {
  float nPer = -1.0;
  
  // clean the sine table
  for (int x = 0 ; x < stimulus_length ; x++) {
    stimulus_table[x] = 0;
  }

  float dur = stimulus_length * (1.0 / f_sampling);

  if (freq < 10000) {
    nPer = (int)(0.5 + dur * freq);
  } else {
    nPer = (int)(0.5 + dur * freq) - 1;
  }
  
  float f_realized = 1.0 / (dur / nPer);
  boolean sat = false;

  float bitAmplitude = A_stimulus / 1.2 * 4090;
  for (int i = 0; i < stimulus_length; i++) {
    int v = DC_offset_stimulus + (int) (0.5 + bitAmplitude * sin(((float)i) * nPer * 2 * PI / ((float)stimulus_length)));
    if (v > 4095) {
      v = 4095;
      sat = true;
    }
    if (v < 0) {
      v = 0;
      sat = true;
    }
    stimulus_table[i] = v;
  }
  
  Serial.print("DEBUG single dur:");
  Serial.print(dur * 1000);
  Serial.print(" ms");
  Serial.print(" Per :");
  Serial.print(nPer);
  Serial.print(" Realized stimulus frequency: ");
  Serial.print(f_realized);
  Serial.print(" Stimulus amplitude: ");
  Serial.print(A_stimulus);
  Serial.print( " (");
  Serial.print(bitAmplitude);
  Serial.print(")");
  Serial.print(" DC offset stimulus:");
  Serial.print(DC_offset_stimulus);
  if (sat)
    Serial.println(" Saturated");
  else
    Serial.println(" ");

  Serial.flush();
  return f_realized;
}




void toggleOnBoardLed() {
  digitalWriteFast(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

// Make sure to call readSingle() to clear the interrupt. FASTRUN
void adc0_isr() {
  //st=micros();
  pOut++;
  pOut = pOut % stimulus_length;
  
  
  analogWrite(pin_writeDAC, stimulus_table[pOut]);
  
  if (p1 >= stimulus_length ) {
    adc->adc0->readSingle();
    adc->adc0->stopPDB();
    dacDisable();
    // Serial.print("ERROR Trying to read beyond end of buffer. ");
    Serial.print("DONE: 0");
    Serial.print(", stimulus_length: ");
    Serial.print(stimulus_length);
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
}

void adc1_isr() {
  if (p2 >= stimulus_length ) {
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
    Serial.print(stimulus_length);
    Serial.print(" Realized stimulus frequency:");
    Serial.print(f_realized);
    Serial.print(" Sampling freq:");
    Serial.println(f_sampling);
  
    Serial.print("#DATA,");
    //send the number of data
    Serial.print(stimulus_length);
    Serial.print(",");
    //send the frequency
    Serial.print(f_realized);
    Serial.print(",");
    //send the acquisition freq
    Serial.println(f_sampling);
    //Serial.print(",");
    Serial.flush();
    } else if (tmp == 1){
      //send all data
      for (int i = 0; i < stimulus_length; i++) {
        Serial.print(V1[i]);
        Serial.print(",");
      }
  } else if (tmp == 2){
      //send all data
      for (int i = 0; i < stimulus_length; i++) {
        Serial.print(V2[i]);
        Serial.print(",");
        }
  } else if (tmp == 3){
      //send all data
      for (int i = 0; i < stimulus_length; i++) {
        Serial.print(stimulus_table[i]);
        Serial.print(",");
        }
  } else {
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
  pinMode(pin_readD, INPUT);
  pinMode(pin_readM, INPUT);
  pinMode(pin_read2, INPUT);
  
  Serial.println("DEBUG Begin setup");


  Serial.flush();
  SIM_SCGC2 |= SIM_SCGC2_DAC0;
  DAC0_C0 = DAC_C0_DACEN;  // 1.2V ref is DACREF_1
  analogReference(0);  // 1: Vref set to 1.2V, 0: Vref set to 3.3 V
  //setAdc0ForAcq();
  //setAdc1ForAcq();
  adc->resetError();
  analogWriteResolution(12);   //values between 0 and 4095
  analogWrite(pin_writeDAC, 1000);
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

  //Start the timer for the On Board LED
  baseTimer.priority(180);
  baseTimer.begin(toggleOnBoardLed, 1000000);

  Serial.println("DEBUG End setup");
  Serial.flush();

}

void loop() {
  delay(100);
}
