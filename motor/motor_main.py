from ctypes import *
import ctypes
import numpy as np
from numpy.ctypeslib import ndpointer
import csv
import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt
import datetime
from datetime import timedelta
import time
import sys
import random
import RPi.GPIO as GPIO
import os
import pigpio
#import calculate_rms

ACTIVE_CHANNELS = 8
PWM_PIN = 19            # GPIO pin 19 for Motor PWM control
MOTOR_EN_PIN = 15       # GPIO pin 15 for Motor enable

data = [[],[],[],[],[],[],[],[],[]]
data_single_revolution = [[],[],[],[]]

kDt = 0.5
kAlpha = 0.01
kBeta = 0.0001

def get_us():
    now = time.perf_counter()
    return now

# returns the elapsed time by subtracting the timestamp provided by the current time 
def get_elapsed_us(timestamp):
    temp = get_us()
    return (temp - timestamp)

class MotorController(object):
    SO_FILE = os.path.dirname(os.path.realpath(__file__)) + "/motor_spi_lib.so"
    C_FUNCTIONS = CDLL(SO_FILE)
    
    def __init__(self, pwm_pin, motor_pin, motor_duration, pwm_target, mode = GPIO.BOARD, freq = 25000, warnings = False):
        GPIO.setwarnings(warnings)
        GPIO.setmode(mode)
        GPIO.setup(motor_pin, GPIO.OUT)
        self.pwm_pin = pwm_pin
        self.motor_pin = motor_pin
        self.pi = pigpio.pi()
        self.motor_duration = motor_duration
        self.pwm_target = pwm_target
        self.INITIAL_US = get_us()
        
        ## Default values
        self.pwm_current = 14
        self.position_hold_time = 0
        self.position_counter = 0
        self.data = []
        self.data_single_revolution = []
        self.last_position = 0
        self.freq_count = [[],[]]
        self.current_rev_time = 0
        self.last_rev_time = 0
        self.master_pos_counter = 0

        self.kX1 = 0.0
        self.kV1 = 0.0
        self.x = []
        self.v = []
        self.r = []


    def initialize(self):
        print("\n*****************************\n")
        msg = ""
        self.pi.hardware_PWM(19, 0, 0)
        GPIO.output(self.motor_pin, 1)
        self.INITIAL_US = get_us()
        _self_check = self.C_FUNCTIONS.initialize_motor()

        if not _self_check:
            print("\nMotor Initialized Successfully\n")

        else:
            ## TODO Raise exception here
            msg = "ERROR: Could not communicate with motor board. Please disconnect motor."
            return 0, msg
        '''
        if input("Would you like to view the registers? (y/n): ").lower() == 'y':
            self._read_registers()

            if(input("\nAre Registers correct? (y/n): ").lower() != 'y'):
                msg = "Registers Not Selected to be Correct."
                return 0, msg
        '''
        if not self.C_FUNCTIONS.initialize_adc():
            print("\nADC Initialized Successfully\n")
        else:
            msg = "ERROR: ADC Initialize Failed. Please Disconnect motor."
            return 0, msg

        return 1, "Initialization complete!"
    
    def analog_in_initial_send(self):
        self.C_FUNCTIONS.getAnalogInAll_InitialSend()

    # Increases PWM control duty cycle by 1%
    # Gets called by run_main until preferred duty cycle is reached
    def pwm_control(self):
        if(self.pwm_current < self.pwm_target):
            self.pwm_current += 1
            print(self.pwm_current)
            #print("PWM: {}".format(self.pwm_current))
        self.pi.hardware_PWM(19, 25000, self.pwm_current * 10000)

    def bcm2835_init_spi(self):
        self.C_FUNCTIONS.AD5592_Init()

    def bcm2835_motor_ping(self):
        GPIO.output(self.motor_pin, 1)
        return self.C_FUNCTIONS.motor_ping()

    def get_analog_data(self):
        return self.C_FUNCTIONS.getAnalogInAll_Receive()
    
    def analog_terminate(self):
        self.C_FUNCTIONS.getAnalogInAll_Terminate()

    def health_check(self, data):
        code = [0,0,0]
        for i in range(1,4): # Turning Hall sensor channel data into a 3-digit position code
            if(data[i] > 1500): # Set a threshold of 1650mV for the hall pulse
                code[i-1] = 1
            else:
                code[i-1] = 0
        #print("Code: {}".format(code))
        position = self._find_positions(code) # Convert code into a position (1-6)
        #print("Position: {}".format(position))
        if(self.last_position != position): # Check if position is different from the last recorded position
            if(self.last_position != 0):
                self.master_pos_counter += 1
                self.position_counter += 1 
                if(self.position_counter == 30):
                    self.current_rev_time = get_us()
                    freq = self._get_rpm(self.current_rev_time, self.last_rev_time)
                    self.running_filter(freq)
                    reluctance = self._motor_reluctance(self.x[-1])
                    #rms_val = self._revolution_rms()
                    self.position_counter = 0
                    self.last_rev_time = self.current_rev_time
                    print('\033c')
                    print("Time: {} ".format(round(get_elapsed_us(self.INITIAL_US), 1)) + "PWM: {} ".format(self.pwm_current) + "RPM: {} ".format(round(freq, 1)))
                    #print('\033c')
                    #print("RPM: {} ".format(freq))
                else:
                    rms_val = 0
                #print("Elapsed: {}, ".format(get_elapsed_us(self.INITIAL_US)) + "Position: {}, ".format(position) + "Frequency: {} ".format(round(freq, 2)) + "Filtered freq: {} ".format(x[-1]) +"PWM: {} ".format(self.pwm_current) + "Freq/PWM = {} ".format(reluctance) + "RMS Current: {}".format(rms_val))
            else:
                pass
                #msg = "INCORRECT POSITION RECORDED"
                #return 0, msg
            self.position_hold_time = get_us()
            self.last_position = position
        else:
            if get_elapsed_us(self.position_hold_time) > 1:
                msg = "STALL DETECTED"
                return 0, msg

        return 1, "All Good!"

    def running_filter(self, data):
        x_k = self.kX1 + kDt * self.kV1
        r_k = data - x_k
        x_k = x_k + kAlpha * r_k
        v_k = self.kV1 + (kBeta/kDt) * r_k

        self.kX1 = x_k
        self.kV1 = v_k

        self.x.append(x_k)
        self.v.append(v_k)
        self.r.append(r_k)        

    def rampdown(self):
        print("Starting rampdown...")
        #self.pi.hardware_PWM(19, 0, 0)
        #return self.C_FUNCTIONS.motor_freewheel()

        for duty in range(self.pwm_current, 0, -1):
            self.pi.hardware_PWM(19, 25000, duty * 10000)
            print("PWM: {}".format(duty))
            time.sleep(0.2)
        self.pi.hardware_PWM(19, 0, 0)
        #GPIO.output(self.motor_pin, 0)
        # graph_data()
        #return 0
        
    def shutdown(self):
    # This occurs when there is a danger event like a stall or overcurrent
    # In this case, we want to shut off everything immediately to prevent further damage
        print("Starting Shutdown")
        self.pi.hardware_PWM(19, 0, 0)
        GPIO.output(self.motor_pin, 0)
        # graph_data()
        #return 0

    def killall(self):
        self.pi.hardware_PWM(19, 0, 0)
        GPIO.output(self.motor_pin, 0)
        #self.pi.close()

    def motor_results(self, resp, msg):
        print("\n\n-----------------------------\n")
        print("-----------------------------\n")
        if not resp:
            print("MOTOR FAILED\n")
            print(msg)
        else:
            print("MOTOR PASSED\n")
        print("\n\n-----------------------------\n")
        print("-----------------------------\n")

    def _read_registers(self):
    # Reads all registers on DRV8343 and prints them
        for i in range(19):
            reg_data = self.C_FUNCTIONS.motor_register_read(i)
            print('Register {}:'.format(i) + ' {}'.format(hex(reg_data)));
            print('\n')

    def _find_positions(self, code):
    # Converts the hall sensor pulse data into a position (1-6)
    # If the hall sensor pulses do not align with one of these positions, a zero is returned at which there will a flag raised
        if code == [1, 0, 1]:
            return 1
        elif code == [0, 0, 1]:
            return 2
        elif code == [0, 1, 1]:
            return 3
        elif code == [0, 1, 0]:
            return 4
        elif code == [1, 1, 0]:
            return 5
        elif code == [1, 0, 0]:
            return 6
        else:
            return 7
    
    def _get_rpm(self, current_rev_time, last_rev_time):

        freq = 60*( 1/((current_rev_time - last_rev_time)*3) )
        self.freq_count[0].append(get_elapsed_us(self.INITIAL_US))
        self.freq_count[1].append(freq)
        return freq

    def _motor_reluctance(self, freq):
        #return freq/self.pwm_current
        return 0
        
    def _revolution_rms(self):
        #TODO: Implement Function here
        return 0

#def graph_freq(MC_1, MC_2, MC_3, MC_4):
def graph_freq(MC_1, MC_2):
    #fig, axs = plt.subplots(2)
    #fig.suptitle('Motor Frequency')
    plt.xlabel('Time (ms)')
    
    #axs[0].set_ylabel(f'Mode 1 RPM @ {MC_1.pwm_target}% target duty')
    #axs[1].set_ylabel(f'Mode 2 RPM @ {MC_2.pwm_target}% target duty')
    
    #axs[0].plot(MC_1.freq_count[0], MC_1.freq_count[1])
    #axs[1].plot(MC_2.freq_count[0], MC_2.freq_count[1])
    
    plt.plot(MC_1.freq_count[0], MC_1.freq_count[1])
    plt.plot(MC_2.freq_count[0], MC_2.freq_count[1])
    #plt.plot(MC_3.freq_count[0], MC_3.freq_count[1])
    #plt.plot(MC_4.freq_count[0], MC_4.freq_count[1])
    #plt.legend([f"TEST1 - PWM target: {MC_1.pwm_target}%", f"TEST2 - PWM target: {MC_2.pwm_target}%", f"TEST3 - PWM target: {MC_3.pwm_target}%", f"TEST4 - PWM target: {MC_4.pwm_target}%"])
    
    plt.legend([f"TEST1 - PWM target: {MC_1.pwm_target}%", f"TEST2 - PWM target: {MC_2.pwm_target}%"])
    plt.show()

def start_sequence():
    print('\033c')
    print("*****************************")
    #print(FILE_OUTPUT_NAME)
    print(f"NURO MOTOR TESTING - {datetime.datetime.now().replace(microsecond=0)}")
    print("*****************************\n")

    MC_start = MotorController(PWM_PIN, MOTOR_EN_PIN, 0, 0)

    MC_start.bcm2835_init_spi()

    print("Waiting on motor board to power up...")
    print("(NOTE: Hold CTRL + 'C' to exit program)\n")

    try:
        while(MC_start.bcm2835_motor_ping()):
            pass
        #print('\033c')
        print("*****************************")
        print("Motor Board Connected!")
        print("*****************************")

        #end_sequence(MC_start)
        
        return 1

    except KeyboardInterrupt:
        end_sequence(MC_start)

        return 0

def end_sequence(MC):
    MC.killall()

def run_motor(MC, file):
    temp_data = np.uint32([0,0,0,0,0,0,0,0,0])
    adc_reading = 0x0
    index = 0x0
    pwm_counter = 0

    resp, msg = MC.initialize()
    if not resp:
        end_sequence(MC)
        return -1, msg

    MC.analog_in_initial_send()

    MC.position_hold_time = MC.revolution_hold_time = get_us()

    while(1):
        if(MC.pwm_current < MC.pwm_target):                              # Ramps up PWM
            if( (pwm_counter == 0) or ((pwm_counter % 1000) == 0) ):
                MC.pwm_control()
            pwm_counter += 1

        for i in range(0, ACTIVE_CHANNELS):
            data_16bit = MC.get_analog_data() 
            adc_reading, index = data_process(data_16bit)
            temp_data[index+1] = adc_reading
            data[index+1].append(temp_data[index+1])

        temp_data[0] = int(round(get_elapsed_us(MC.INITIAL_US), 6) * 1000000)
        data[0].append(temp_data[0])
        writer = csv.writer(file)
        writer.writerow(temp_data)

        try:
            resp, msg = MC.health_check(temp_data)
            if not resp:
                MC.analog_terminate()
                MC.shutdown()
                return -1, msg
            if(temp_data[0] >= MC.motor_duration * 1000000):
                MC.analog_terminate()
                #print(hex(MC.rampdown()))
                #time.sleep(10)
                
                MC.rampdown()
                
                msg = "Motor duration reached: {}".format(temp_data[0])
                return 1, msg
        except KeyboardInterrupt:

            MC.analog_terminate()
            MC.rampdown()
            msg = "----Keyboard Interrupt by user----"
            return -1, msg

        finally:
            pass

def data_process(data):
    index = ((data >> 12) & 0x7)
    data_converted = int(data & 0xFFF) * (5000/4095)
    if index in range(0,3): # Channels 0-2 are hall sensors - use voltage translation
        adc_reading = data_converted
    elif index in range(3,6): # Channes 3-5 are current sensors - use current translation
        #adc_reading = (3000 - data_converted)
        if data_converted >= 3000:
            adc_reading = 0
        else:
            adc_reading = (10 * (3000 - data_converted))
    elif index in range(6,9):
        adc_reading = data_converted
        #adc_reading = int(((data_converted - 409.5) * 0.7535795) + 25)
    return adc_reading, index
    #return data_converted, index

def message_display(msg, desired_answer):
    while(1):
        if input(msg).lower() == desired_answer:
            return 1
        else:
            print('\033c')
            print("*****************************")
            print("Incorrect character entered.")
            print("*****************************")
            return 0

def run_main():
        
    FILE_OUTPUT_NAME = str(datetime.datetime.now().replace(microsecond=0))
    file1 = open("/home/pi/Documents/MOTOR_DATA_FOLDER/" + FILE_OUTPUT_NAME + " mode1_test", 'w', newline='')

    print('\033c')
    print("*****************************")
    print("This test will run 2 configurable modes. Please enter parameters below:")
    MOTOR_DURATION_MC1 = int(input("Enter duration 1: "))
    MOTOR_DURATION_MC2 = int(input("Enter duration 2: "))
    #MOTOR_DURATION_MC3 = int(input("Enter duration 3: "))
    #MOTOR_DURATION_MC4 = int(input("Enter duration 4: "))

    MOTOR_PWM_TARGET_MC1 = int(input("Enter target duty cycle 1: "))
    MOTOR_PWM_TARGET_MC2 = int(input("Enter target duty cycle 2: "))
    #MOTOR_PWM_TARGET_MC3 = int(input("Enter target duty cycle 3: "))
    #MOTOR_PWM_TARGET_MC4 = int(input("Enter target duty cycle 4: "))

    MC_1 = MotorController(PWM_PIN, MOTOR_EN_PIN, MOTOR_DURATION_MC1, MOTOR_PWM_TARGET_MC1)
    
    resp, msg = MC_1.initialize()
    if not resp:
        end_sequence(MC_1)
        return -1
    MC_2 = MotorController(PWM_PIN, MOTOR_EN_PIN, MOTOR_DURATION_MC2, MOTOR_PWM_TARGET_MC2)
    #MC_3 = MotorController(PWM_PIN, MOTOR_EN_PIN, MOTOR_DURATION_MC3, MOTOR_PWM_TARGET_MC3)
    #MC_4 = MotorController(PWM_PIN, MOTOR_EN_PIN, MOTOR_DURATION_MC4, MOTOR_PWM_TARGET_MC4)
    
    print('\033c')
    print("----PLEASE CONNECT MOTOR----\n")
    
    try:
        while(message_display("Once motor is connected please press 'y' and ENTER: ", 'y') != 1):
            pass
        print('\033c')
        print("*****************************")
        print("----Testing Mode 1----")

        resp1, msg1 = run_motor(MC_1, file1)
        print(msg1)
        #end_sequence(MC_1)
        if resp1 < 0:
            #print('\033c')
            print(msg1)
            while(message_display("\nType 'c' and ENTER to continue: ", 'c') != 1):
                pass
            print('\033c')
            print("\nRestarting test program...")
            time.sleep(3)
            return -1
        MC_1.motor_results(resp1, msg1)
        time.sleep(2)
        #print('\033c')
        print("*****************************\n")
        print("----Testing Mode 2----")
        
        file2 = open("/home/pi/Documents/MOTOR_DATA_FOLDER/" + FILE_OUTPUT_NAME + " mode2_test", 'w', newline='')
        resp2, msg2 = run_motor(MC_2, file2)
        print(msg2)
        #end_sequence(MC_2)
        if resp2 < 0:
            #print('\033c')
            print(msg2)
            while(message_display("\nType 'c' and ENTER to continue: ", 'c') != 1):
                pass
            print('\033c')
            print("Restarting test program...")
            time.sleep(3)
            return -1
        #time.sleep(5)
        print("*****************************\n")
        print("----Testing Mode 3----")
        '''
        file3 = open("/home/pi/Documents/MOTOR_DATA_FOLDER/" + FILE_OUTPUT_NAME + " mode3_test", 'w', newline='')
        resp3, msg3 = run_motor(MC_3, file3)
        print(msg3)
        #end_sequence(MC_2)
        if resp3 < 0:
            #print('\033c')
            print(msg3)
            while(message_display("\nType 'c' and ENTER to continue: ", 'c') != 1):
                pass
            print('\033c')
            print("Restarting test program...")
            time.sleep(3)
            return -1
        time.sleep(5)
        print("*****************************\n")
        print("----Testing Mode 4----")
        
        file4 = open("/home/pi/Documents/MOTOR_DATA_FOLDER/" + FILE_OUTPUT_NAME + " mode4_test", 'w', newline='')
        resp4, msg4 = run_motor(MC_4, file4)
        print(msg2)
        #end_sequence(MC_2)
        if resp4 < 0:
            #print('\033c')
            print(msg2)
            while(message_display("\nType 'c' and ENTER to continue: ", 'c') != 1):
                pass
            print('\033c')
            print("Restarting test program...")
            time.sleep(3)
            return -1
        '''
        
        rms1, rms2 = calculate_rms(FILE_OUTPUT_NAME + " mode1_test", FILE_OUTPUT_NAME + " mode2_test")

        print(f"Phase RMS for mode1 [A, B, C]: {rms1}")
        print(f"Phase RMS for mode2 [A, B, C]: {rms2}")
        
        MC_2.motor_results(resp2, msg2)
        
        #graph_freq(MC_1, MC_2)
        #graph_freq(MC_1, MC_2, MC_3, MC_4)

        #print('\033c')
        print("Please disconnect motor!\n")
        while( message_display("Press 'c' and ENTER to continue to next motor, or CTRL + 'C' to exit program: ", 'c') != 1):
            pass
        time.sleep(1)
        return 1
    except KeyboardInterrupt:
        end_sequence(MC_1)
        end_sequence(MC_2)
        #end_sequence(MC_3)
        #end_sequence(MC_4)
        return 0

if __name__ == "__main__":
    while(1):
        if start_sequence() == 0:
            sys.exit()

        while(1):
            state = run_main()

            if state == 0 :
                print('\033c')
                print("*****************************")
                print("This program will be shutting down in 3 seconds")
                print("*****************************")
                time.sleep(3)
                sys.exit()

            elif state == -1:
                break

            else:
                pass


