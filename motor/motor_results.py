import csv
import matplotlib.pyplot as plt
import numpy as np
import math

class RMS_calc(object):

    def __init__(self, filename, file_start):
        self.filename = filename
        self.file_start = file_start
        self.rms = []
        self.x = []
        self.y = [[],[],[],[]]

    def collect_data(self):
        with open("/home/pi/Documents/FLEX_DATA_FOLDER/" + self.filename, 'r') as csvfile:
            plots = csv.reader(csvfile,delimiter=',')
            #next(plots)
            for row in plots:
                self.x.append(int(row[0]))
                for i in range(1,5):
                    self.y[i-1].append(int(row[i]))
    #print(x)

    def test_rpm(self):
        rpm_max = 0
        rpm_min = 1000000
        temp_sum = 0
        rpm_data = []
        for i in range(0, len(self.x)):
            temp_sum += self.y[1][i]

            if rpm_min > self.y[1][i]:
                rpm_min = self.y[1][i]
            if rpm_max < self.y[1][i]:
                rpm_max = self.y[1][i]
        avg = round(temp_sum / i, 1)
        rpm_data.append(rpm_min)
        rpm_data.append(rpm_max)
        rpm_data.append(avg)
        return rpm_data

    def test_current(self):
        i_data = [[],[],[]]
        for k in range(2, 5):
            i_max = 0
            i_min = 1000000

            for j in range(0, len(self.x)):

                if i_min > self.y[k][j]:
                    i_min = self.y[k][j]
                if i_max < self.y[k][j]:
                    i_max = self.y[k][j]

            i_data[k-2].append(rpm_min)
            i_data[k-2].append(rpm_max)

        return i_data

def main(filename_1, filename_2, file1_start, file2_start):
    test1 = RMS_calc(filename_1, file1_start)
    test2 = RMS_calc(filename_2, file2_start)

    test1.collect_data()
    test2.collect_data()

    rpm_1 = test1.test_rpm()
    rpm_2 = test2.test_rpm()

    current_1 = test1.test_current()
    current_2 = test2.test_current()

    return rpm1, current_1, rpm_2, current_2
