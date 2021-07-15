import csv
import matplotlib.pyplot as plt
import numpy as np
import math

x = []
y = [[],[],[]]

rms = []

display_num = 3
fileName = ''

def collect_data():
    with open("/home/pi/Documents/MOTOR_DATA_FOLDER/" + fileName, 'r') as csvfile:
        plots = csv.reader(csvfile,delimiter=',')
        next(plots)
        for row in plots:
            x.append(int(row[0]))
            for i in range(4,7):
                y[i-4].append(int(row[i]))
    #print(x)

def rms_calc():
    for i in range(0, 3):
        temp_sum = 0
        temp_rms = 0
        for j in range(1,len(y[0])):
            temp_sum += (2 * ((y[i][j])**2) * (x[j] - x[j-1]))
        temp_rms = temp_sum/(x[len(y[0]) - 1] - x[0])
        rms.append(round((math.sqrt(temp_rms))/1000, 3))

if __name__ == "__main__":
    fileName = input("Enter filename:")
    collect_data()
    rms_calc()
    
    print(f"\nPhase A RMS: {rms[0]}")
    print(f"Phase B RMS: {rms[1]}")
    print(f"Phase C RMS: {rms[2]}")
    
