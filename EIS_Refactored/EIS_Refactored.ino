;// The code in this sketch is based on code provided by the authors of the paper:
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

boolean sinetable_generated = 0;
float f_sampling = 0;

const int SKIP = 1000;
int skipped = 0;
int averaging_number = 1;

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

void setStimulusFrequency(){
    char buf[100];
    //this is an output frequency request
    commandBuffer[0] = ' ';
    float tmp = atof(commandBuffer);
    f_requested = tmp;
    sprintf(buf, "Stimulus frequency set to: %f", f_requested);
    Serial.println(buf);
}

void setStimulusAmplitude(){
    char buf[100];
    //this is an amplitude out change request
    commandBuffer[0] = ' ';
    float tmp = atof(commandBuffer);
    if (((tmp >= 0) & (tmp <= 1))) {
      A_stimulus = tmp;
      sprintf(buf, "Stimulus amplitude set to %f", A_stimulus);
      Serial.println(buf);
    } else {
      sprintf(buf, "E05 Stimulus amplitude out of range, amplitude given: %f, maximum allowed: %f", tmp, 1.0);
      Serial.println(buf);
    }
}

void setStimulusDCValue(){
  char buf[100];
  //this is a zero change request
      commandBuffer[0] = ' ';
      float tmp = atof(commandBuffer);
      if (((tmp >= 0) & (tmp <= 4095))) {
        DC_offset_stimulus = (int)tmp;
        sprintf(buf, "Stimulus DCValue set to %d", DC_offset_stimulus);
        Serial.println(buf);
      } else {
        sprintf(buf, "E05 Stimulus DCValue requested:  %f is out of range maximum value allowed: %f", tmp, 4095.0);
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
      setStimulusFrequency();
    } else if (commandBuffer[0] == 'A') {
      setStimulusAmplitude();
    } else if (commandBuffer[0] == 'Y') {
      setStimulusDCValue();
    }  else if (commandBuffer[0] == 'G') {
      setSamplingFrequency();
    }  else if (commandBuffer[0] == 'M') {
      startMeasurement(f_requested, f_sampling);
    } else if (commandBuffer[0] == 'Z') {
      Serial.println("DEBUG: measureOCP disabled ");
      // measureOCP();
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
  DAC0_C0 = DAC_C0_DACEN  | DAC_C0_DACRFS;   // 1.2V ref is DACREF_1
  pinMode(pin_writeDAC, OUTPUT);
}

/*
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
}*/

void startMeasurement(float f_requested, float f_sampling) {

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

  sinetable_generated = loadSineTable(f_requested, f_sampling);
  Serial.print("DEBUG Start pdb with frequency ");
  Serial.print(f_sampling);
  Serial.print(" Hz.");
  Serial.print(" Sinetable generated: ");
  Serial.println(sinetable_generated);
  Serial.flush();
  
  if (f_requested <= 0.01*f_sampling) {
    averaging_number = 4;
  } else {
    averaging_number = 1;
  }

  adc->adc0->setAveraging(averaging_number); // set number of averages
  adc->adc1->setAveraging(averaging_number); // set number of averages

  adc->adc0->stopPDB();
  //adc->adc0->startSingleRead(pin_readD);// call this to setup everything before the pdb starts, differential is also possible
  adc->adc0->startSingleDifferential(pin_readD, pin_readM);// call this to setup everything before the pdb starts, differential is also possible
  adc->adc0->enableInterrupts(adc0_isr, 200);
  adc->adc0->startPDB(f_sampling); //frequency in Hz
  adc->adc1->stopPDB();
  adc->adc1->startSingleRead(pin_read2); // call this to setup everything before the pdb starts
  adc->adc1->enableInterrupts(adc1_isr, 201);
  adc->adc1->startPDB(f_sampling); //frequency in Hz

}


boolean loadSineTable(float f_stimulus, float f_sampling) {
  boolean DAC_saturated = false;
  boolean f_sampling_sufficient = true;
  
  float duration = stimulus_length * (1.0 / f_sampling);

  if (f_sampling < 3 * f_stimulus ) {
    Serial.print("Error: f_sampling is below the minimal required sampling frequency: 1.5*2*f_stimulus");
    f_sampling_sufficient = false;
  };

  int A_max = min(DC_offset_stimulus, 4095-DC_offset_stimulus) - 1;
  
  float amplitude = A_stimulus * A_max;
  float omega_dt =  f_stimulus * 2 * PI / ((float)stimulus_length);  // phase change over one sample

  if (amplitude > A_max) {
      DAC_saturated = true;
  };

  int v_dac;
  for (int i = 0; i < stimulus_length; i++) {
    v_dac = DC_offset_stimulus + (int) (0.5 + amplitude * sin(((float)i) *omega_dt ));
    if (v_dac > 4095)  { v_dac = 4095; } else if (v_dac < 0)  {v_dac = 0;};  // Prevent overflow
    stimulus_table[i] = v_dac;
  }

  Serial.print("stimulus duration:");
  Serial.print(duration * 1000);
  Serial.println(" ms");
  
  Serial.print("stimulus amplitude: ");
  Serial.print(A_stimulus);
  Serial.print( " (");
  Serial.print(amplitude);
  Serial.println(")");
   
  Serial.print("DC offset stimulus:");
  Serial.println(DC_offset_stimulus);
  
  Serial.print("DAC Saturated: ");
  Serial.println(DAC_saturated);
  
  Serial.print("Sampling rate consistent with sampling theorem: ");
  Serial.println(f_sampling_sufficient);

  Serial.flush();
  return f_sampling_sufficient and ~DAC_saturated;
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
  size_t bytes_written = 1;

  if (tmp == 0){
    Serial.print("Stimulus length:");
    Serial.print(stimulus_length);
    
    Serial.print(" Stimulus frequency:");
    Serial.print(f_requested);
    
    Serial.print(" Sampling frequency:");
    Serial.println(f_sampling);

    Serial.print("ADC Averaging_number:");
    Serial.println(averaging_number);

    Serial.print("#DATA,");
    //send the number of data
    Serial.print(stimulus_length);
    Serial.print(",");
    //send the frequency
    Serial.print(f_requested);
    Serial.print(",");
    //send the acquisition freq
    Serial.println(f_sampling);
    //Serial.print(",");
    Serial.flush();
    } else if (tmp == 1){
      //send all data
      for (int i = 0; i < stimulus_length and bytes_written > 0; i++) {
        bytes_written = Serial.print(V1[i]);
        bytes_written += Serial.print(",");
      }
  } else if (tmp == 2){
      //send all data
      for (int i = 0; i < stimulus_length and bytes_written > 0; i++) {
        bytes_written = Serial.print(V2[i]);
        bytes_written += Serial.print(",");
        }
  } else if (tmp == 3){
      //send all data
      for (int i = 0; i < stimulus_length and bytes_written > 0; i++) {
        bytes_written = Serial.print(stimulus_table[i]);
        bytes_written += Serial.print(",");
        }
  } else {
      Serial.print("Future extension aka Not Implemented");
  }

  //tag the end of sending
  Serial.print("#END");
  Serial.flush();
}


void setup() {

  Serial.begin(115000);
  while (!Serial) ;  // Wait for a serial connection
  Serial.println("DEBUG Begin setup");
  Serial.flush();

  // Set pin modes
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pin_readD, INPUT);
  pinMode(pin_readM, INPUT);
  pinMode(pin_read2, INPUT);

  // Configure DA-converter (see analog.c and kinetis.h)
  dacEnable();
  analogWriteResolution(12);              // Define range of allowed values: 12 bits => range = [0, 4095]
  analogReference(0);    //  0: Vref set to 3.3 V, 1: Vref set to 1.2V. NB: This affects ADC and DAC!
  adc->resetError();

  // delay(1000);
  // analogWrite(pin_writeDAC, 4095);     // Test led in external circuit ....
  // delay(1000);
  analogWrite(pin_writeDAC, 0);
  adc->resetError();
  dacDisable();


  // Start the timer for the On Board LED
  // https://www.pjrc.com/teensy/td_timing_IntervalTimer.html
  baseTimer.priority(180);
  baseTimer.begin(toggleOnBoardLed, 1000000);   // interval is specified in microseconds

  Serial.println("DEBUG End setup");
  Serial.flush();
}

void loop() {
  delay(100);
}
