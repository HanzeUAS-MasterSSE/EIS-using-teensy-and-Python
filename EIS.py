import serial
import numpy as np
from time import sleep
import sys
import json


def estimate_impedance(Rs, f_stim, f_samp, V1, V2, DAC):
    ''' Estimate the frequency response at the stimulus frequency.
        
        Parameters:
            Rs: shunt resistance
            f_stim: stimulus frequency
            f_samp: sampling frequency
            V1: measured V1 data
            V2: measured V2 data
            DAC: signal provided to DAC
            
        Output:
            Z: impedance at frequency f_stim
            V1_1, V1_2, V1_0: fit parameters for equation 6 from referenced paper
            V2_1, V2_2, V2_0: fit parameters for equation 7 from referenced paper
        
        'Electrochemical Impedance Spectroscopy System Based on a Teensy Board',
        Leila Es Sebar, Leonardo Iannucci, Emma Angelini, Sabrina Grassini, and Marco Parvis,
        IEEE TRANSACTIONS ON INSTRUMENTATION AND MEASUREMENT, VOL. 70, 2021

        Here we implement equations 6, 7 and 8 of this paper.

        Still needs a resistor and stimulus amplitude correction.

    '''
    
    
    if not len(V1) == len(V2) or not len(V1) == len(DAC):
        
        raise ValueError(f'Incompatible input lengths: len V1: {len(V1)}, len V2: {len(V2)}, len Dac: {len(DAC)}')
        
    tstamps = np.arange(0, len(DAC), 1)/f_samp
    
    sine = np.sin(2*np.pi*f_stim*tstamps)
    cosine = np.cos(2*np.pi*f_stim*tstamps)
    
    A = np.vstack([cosine, sine, np.ones(len(DAC))]).T
    V1_1, V1_2, V1_0 = np.linalg.lstsq(A, V1, rcond=None)[0]  # Fit according equation 6
    V2_1, V2_2, V2_0 = np.linalg.lstsq(A, V2, rcond=None)[0]  # Fit according equation 7
    
    Z = (V2_2 + 1j* V2_1)/(V1_2 + 1j* V1_1)*Rs/2 # Apply equation 8, factor is difference between differential and direct on teensy
    
    return Z, V1_1, V1_2, V1_0, V2_1, V2_2, V2_0 



class EIS():
    
    adc_np_type_map = {1:np.int16,2:np.uint16,3:np.uint16}
    
    def __init__(self, COM, BAUD = 115200, timeout = .1):
        self.serial = serial.Serial(COM, BAUD, timeout = .1)

    def get_and_print_responses(self, print_response = False):
        '''
            Retrieve messages written by the teensy to the 
            serial connection under the assumption that the output is 
            utf-8 encoded. If this assumption is violated the resulting 
            exception is printed and an empty string is returned.
        '''
        response = 'No response'
        ret = ''
        while response != '':
            response  = self.serial.readline()
            try:
                response = response.decode('utf-8') 
                ret += response   
            except Exception as e:
                print(f'Exception: {e} \nResponse: {response}')
                ret = ''
                
            if print_response:
                print(response)
        return ret

    def get_data(self, stimulus_parameters, adc_type):
        '''
        This function can be used to get all the measured data for a single subtype.
        
        Parameters:
            
            stimulus_parameters: contains metadata on the stimulus.
            
            adc_type: 1. V1 
                      2. V2
                      3. DAC stimulus
            
        Output:
        
            data: a numpy array of the correct type (see adc_np_type_map) containing
            the full measurement sequence for a single type. 
        '''
                
        start_pos = 0
        dtype = self.adc_np_type_map[adc_type]
        length = stimulus_parameters['length']

        # Allocate memory to store the result
        data = np.zeros((length,), dtype=dtype)
        
        while True:
            data_slice = self.get_data_slice(adc_type, start_pos) 
            
            if data_slice['end']<=data_slice['start']: # This signals that we try to read beyond the end of the avaialble data
                break
                
            data[data_slice['start']:data_slice['end']]=np.array(data_slice['data'],dtype=dtype)
            
            start_pos = data_slice['end']    
            
        return data        

    def get_data_slice(self, adc_type, start_pos):
        '''
        This function can be used to get a slice of the measured data.
        
        Parameters:
                
            adc_type: 1. V1 
                      2. V2
                      3. DAC stimulus
                
            start_pos: sample at which to start the data request. 
            
        Output:
        
            dataslice: A dictionary containing three key-value pairs: 'start', 'end' 
                             and 'data'. The value of 'start' is the value of the parameter 
                             start_pos, the value of 'end' is the index of the first sample 
                             that is not returned (to be used as start_pos in the next call), 
                             the value of the 'data' property  contains the measured data 
                             as a list.
        '''
                     
        commandstring = f'D{adc_type:2}_{start_pos}\n'
        self.serial.write(commandstring.encode('utf8'))
        
        json_data = self.get_and_print_responses( print_response = False)
        data_slice = json.loads(json_data)
        
        return data_slice   


    def get_stimulus_parameters(self):
        '''
        This function can be used to get all the stimulus parameters.
               
        Output:
        
            stimulus_parameters: 
                dictionary with metadata on the stimulus, available keys:
                     'stimulus_parameters_valid': 
                            0 (False) or 1 (True) indicates whether 
                            data in buffers are from measurement for these 
                             stimulus settings
                     'length': 
                            length of the stimulus and the measured data
                     'digital_amplitude':
                            Amplitude of DAC input
                     'f_stimulus': 
                            stimulus frequency DAC
                     'f_sampling': 
                            sampling frequency DAC and ADC
                     'stimulus_duration': 
                            stimulus duraction in seconds
                     'ADC_averaging_number': 
                            averaging over samples in ADC
        '''
        
        jsonStimulusParameters = None
        stimulus_parameters = None
        try:
            # Get the metadata
            self.serial.write("D00\n".encode('utf8'))
            jsonStimulusParameters = self.get_and_print_responses()
            stimulus_parameters = json.loads(jsonStimulusParameters)
        except Exception as e:
            print("jsonStimulusParameters: ", jsonStimulusParameters)
            print("stimulus_parameters: ",stimulus_parameters)
            raise(e)
        
        
        if not stimulus_parameters['stimulus_parameters_valid']:
            raise RuntimeError(f"Stimulus parameters where changed after the last measurement: {stimulus_parameters}")
            
        
        return stimulus_parameters 



    def set_stimulus_parameters(self, 
                        f_stimulus, 
                        DC_offset = 2048,
                        A_stimulus = 0.6,
                        f_sampling = 10000,    
                        print_response = False):
                            
        ''' Here rshould be a help string
        
        '''
        
         # "A" Change the output amplitude used for the measurement
        self.serial.write(f"A{A_stimulus}\n".encode('utf8'))
        response = self.get_and_print_responses()
        
        if print_response:
            print(response)    
        
        # "G<samplefreq>\n" Acquire data at <samplefreq>
        self.serial.write(f"G{f_sampling}\n".encode('utf8'))
        response = self.get_and_print_responses()
        
        if print_response:
            print(response) 
        
        # "Y<DC_Offset>\n" set average value stimulus (has to be between positive)
        self.serial.write(f"Y{DC_offset}\n".encode('utf8'))
        response = self.get_and_print_responses()
        
        if print_response:
            print(response) 
            
        # Set the stimulus frequency
        measurement_string = f"F{f_stimulus}\n"
        self.serial.write(measurement_string.encode('utf8'))
        response = self.get_and_print_responses()
        
        if print_response:
            print(response)
        
        
        
    def measure_spectrum(self, 
                        f_range,
                        DC_offset = 2048,
                        A_stimulus = 0.6,
                        f_sampling = 10000,
                        Rs= 1000,
                        process_measurement=estimate_impedance ):
        ''' Here rshould be a help string
        
        '''
        
       
        spectrum = [None]*len(f_range)
        
        for f_index, f in enumerate(f_range):
            print(f"\n--------------------------------------------")
            
            self.set_stimulus_parameters(f, 
                                                                DC_offset = DC_offset,
                                                                A_stimulus = A_stimulus,
                                                                f_sampling = f_sampling,
                                                                print_response = True)
            
            print(f"Measure at frequency {f}")
            
            # Execute Measurement
            self.serial.write("M\n".encode('utf8'))
            sleep(0.1)
            response = self.get_and_print_responses()
        
            while True:
                sleep(0.1)
                # Get the metadata
                self.serial.write("D00\n".encode('utf8'))
                jsonStimulusParameters = self.get_and_print_responses()
                try:
                    stimulus_parameters = json.loads(jsonStimulusParameters)
                except Exception as e:
                    print('JSON STRING:', jsonStimulusParameters)
                    raise e
                    
                print('.',end='')
                # Check if measurement is finished
                if stimulus_parameters['stimulus_parameters_valid']: 
                    break
            
            V1 = self.get_data(stimulus_parameters, 1)
            V2 = self.get_data(stimulus_parameters, 2)
            DAC = self.get_data(stimulus_parameters, 3)  
            
            spectrum[f_index] = process_measurement(Rs, f, f_sampling, V1, V2, DAC)
            # sleep(0.1)
            print(stimulus_parameters)
        
        print(f"\n--------------------------------------------")
        return spectrum
