;// The code in this sketch is based on code provided by the authors of the paper:
//   'Electrochemical Impedance Spectroscopy System Based on a Teensy Board',
//   Leila Es Sebar, Leonardo Iannucci, Emma Angelini, Sabrina Grassini, and Marco Parvis,
//   IEEE TRANSACTIONS ON INSTRUMENTATION AND MEASUREMENT, VOL. 70, 2021
//
// It was refactored and rewritten for use with the Python serial module by:
//      Ronald A.J.  van Elburg (www.vanelburg.eu)

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
float f_stimulus = 50;   // stimulus frequency
float A_stimulus = 0.2;     // stimulus amplitude
int DC_offset_stimulus = 2000;               // stimulus DC offset
boolean stimulus_parameters_valid = false;

float digital_amplitude;  // these are calculated
float stimulus_duration;  // these are calculated

boolean sinetable_generated = 0;
float f_sampling = 48000;

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
        analogWrite(pin_writeDAC, tmp);
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

        delay(10);
        
        //make some reads
        int x0d = 0;
        unsigned int x1s = 0;
        int noofmeasurments = 500;   
        
        for (int i = 0; i < noofmeasurments; i++) {
          int x0dv = adc->adc0->analogReadDifferential(pin_readD, pin_readM);
          unsigned int x1sv = adc->adc1->analogRead(pin_read2);
          x0d += x0dv;
          x1s += x1sv;
        }
        
        x0d /= noofmeasurments;
        x1s /= noofmeasurments;

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
    f_stimulus = tmp;
    sprintf(buf, "Stimulus frequency set to: %f", f_stimulus);
    Serial.println(buf);
    
     stimulus_parameters_valid = false;
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
    
     stimulus_parameters_valid = false;
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
      
      stimulus_parameters_valid = false;
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
  
  stimulus_parameters_valid = false;
  
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
      startMeasurement(f_stimulus, f_sampling);
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
    if (bufferPos < SERIAL_BUFFER_SIZE-1) {
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

void startMeasurement(float f_stimulus, float f_sampling) {
  String metadata_json;
  //clear inputs just in case
  for (int i = 0; i < stimulus_length; i++) {
    V1[i] = 0;
    V2[i] = 0;
  }

  dacEnable();

  //be sure the adc is set for acquire
  setAdc0ForAcq();
  setAdc1ForAcq();

  p1 = 0;
  p2 = 0;
  pOut = 0;

  sinetable_generated = loadSineTable(f_stimulus, f_sampling);
  
  if (sinetable_generated){
        metadata_json = jsonStimulusParameters();
        Serial.print(metadata_json);
        Serial.flush();
    } else {
       Serial.print("SineTable generated with errors. Check amplitude and  sampling fequency!");
       metadata_json = jsonStimulusParameters();
       Serial.print(metadata_json);
       Serial.flush();
    };
 

  if (f_stimulus <= 0.01*f_sampling) {
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

  stimulus_duration = stimulus_length * (1.0 / f_sampling);

  if (f_sampling < 3 * f_stimulus ) {
    // Serial.print("Error: f_sampling is below the minimal required sampling frequency: 1.5*2*f_stimulus");
    f_sampling_sufficient = false;
  };

  int A_max = min(DC_offset_stimulus, 4095-DC_offset_stimulus) - 1;

  digital_amplitude = A_stimulus * A_max;
  float omega_dt =  f_stimulus * 2 * PI / ((float) f_sampling);  // phase change over one sample

  if (digital_amplitude > A_max) {
      DAC_saturated = true;
  };

  int v_dac;
  for (int i = 0; i < stimulus_length; i++) {
    v_dac = DC_offset_stimulus + (int) (0.5 + digital_amplitude * sin(((float)i) *omega_dt ));
    if (v_dac > 4095)  { v_dac = 4095; } else if (v_dac < 0)  {v_dac = 0;};  // Prevent overflow
    stimulus_table[i] = v_dac;
  }

  /* Serial.print("stimulus stimulus_duration:");
  Serial.print("DAC Saturated: ");
  Serial.println(DAC_saturated);

  Serial.print("Sampling rate consistent with sampling theorem: ");
  Serial.println(f_sampling_sufficient);

  Serial.flush();*/
  
  stimulus_parameters_valid = false;
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
    adc->adc0->disableInterrupts();
    adc->adc0->readSingle();
    adc->adc0->stopPDB();
    dacDisable();
    stimulus_parameters_valid = true;
    // Serial.print("ERROR Trying to read beyond end of buffer. ");
    Serial.print("DONE: 0");
    Serial.print(", stimulus_length: ");
    Serial.print(stimulus_length);
    Serial.print(", p1: ");
    Serial.print(p1);
    Serial.flush();
    return;
  }

  V1[p1++] = adc->adc0->readSingle(); // read a new value, will return ADC_ERROR_VALUE if the comparison is false.
}

void adc1_isr() {
  if (p2 >= stimulus_length ) {
    adc->adc1->disableInterrupts();  
    adc->adc1->readSingle();
    adc->adc1->stopPDB();
    dacDisable();
    Serial.println("DONE: 1");
    Serial.flush();
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


String jsonStimulusParameters(){
    String jsonStimPars("");
    jsonStimPars += "{";
 
      jsonStimPars += "\"stimulus_parameters_valid\":";
          jsonStimPars += stimulus_parameters_valid;
      jsonStimPars += ",";
      
      jsonStimPars += "\"length\":";
          jsonStimPars += stimulus_length;
      jsonStimPars += ",";

      jsonStimPars += "\"digital_amplitude\":";
          jsonStimPars += digital_amplitude;
      jsonStimPars += ",";

      jsonStimPars += "\"f_stimulus\":";
          jsonStimPars += f_stimulus;
      jsonStimPars += ",";

      jsonStimPars += "\"f_sampling\":";
          jsonStimPars += f_sampling;
      jsonStimPars += ",";

      jsonStimPars += "\"stimulus_duration\":";
          jsonStimPars += stimulus_duration * 1000;
      jsonStimPars += ",";

      jsonStimPars += "\"ADC_averaging_number\":";
          jsonStimPars += averaging_number;

    jsonStimPars += "}";

    return jsonStimPars;
}

void sendData() {
    char type[3];
    int type_index;

    int start_position;
    int end_position;
    const int slice_length = 100;

    boolean use_comma = false;

    // Extract type index
    type[0]=commandBuffer[1];
    type[1]=commandBuffer[2];
    type[2]=0;
    type_index = atoi(type);

    if (type_index == 0){
        String metadata_json = jsonStimulusParameters();
        Serial.print(metadata_json);
    } else {
        //this is an sampling start request
        for  (int i = 0; i < 4; i++){ commandBuffer[i]=' '; }  // Remove already processed chars.

        start_position = atoi(commandBuffer);
        end_position = min( start_position + slice_length,  stimulus_length);

        Serial.print("{\"start\":");
        Serial.print(start_position);
        Serial.print(", \"end\":");
        Serial.print(end_position);
        Serial.print(", \"data\": [");

        use_comma = false;

        if (type_index == 1){
            for (int i = start_position; i < end_position; i++) {
                if (use_comma) {Serial.print(",");} else {use_comma=true;};
                Serial.print(V1[i]);
            }
        } else if (type_index == 2){
            for (int i = start_position; i < end_position; i++) {
              if (use_comma) {Serial.print(",");} else {use_comma=true;};
              Serial.print(V2[i]);
              }
        } else if (type_index == 3){
            for (int i = start_position; i < end_position; i++) {
              if (use_comma) {Serial.print(",");} else {use_comma=true;};
              Serial.print(stimulus_table[i]);
              }
        } else {
            Serial.print("\"unrecognized type\"");
        }

        Serial.println("]}");
    }

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
