import csv
import matplotlib.pyplot as plt
import numpy as np

x = []
y = [[],[],[]]

rms = []

display_num = 3
fileName = ''

def collect_data():
    with open(fileName, 'r') as csvfile:
        plots = csv.reader(csvfile,delimiter=',')
        next(plots)
        for row in plots:
            x.append(int(row[0]))
            for i in range(4,7):
                y[i-4].append(int(row[i]))
    print(x)

def rms_calc():
    temp_sum = 0
    temp_rms = 0
    for i in range(0, 3):
        for j in range(1,len(y[0])):
            sum += (2 * ((y[i][j])**2) * (x[j] - x[j-1]))
        temp_rms = sum/(x[len(y[0]) - 1] - x[0])
        rms.append(temp_rms)

if __name__ == "__main__":
    fileName = input("Enter filename:")
    collect_data()
    rms_calc()
    print(f"RMS: {rms}")
    
