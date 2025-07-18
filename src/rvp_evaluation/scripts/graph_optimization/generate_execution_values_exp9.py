import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import figaspect
import matplotlib.ticker as mtick
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from scipy.stats import mannwhitneyu
import csv
import os
import bisect
from collections import OrderedDict
import statistics
from collections import Counter


def readValues1(filename, columns):
    try:
        with open(filename, 'r') as csvfile:
            csv_reader = csv.reader(csvfile, delimiter=',')
            column_names = next(csv_reader)

            value_names = [column_names[i] for i in columns]
            values = [[] for _ in range(len(columns))]

            for row in csv_reader:
                for i in range(len(columns)):
                    values[i].append(float(row[columns[i]]))

    except IOError:
        print('File \'{}\' could not be opened; skipping file'.format(filename))
        return [], [], [], []
    
    return values


def readValues2(filename, columns):
    try:
        with open(filename, 'r') as csvfile:
            csv_reader = csv.reader(csvfile, delimiter=',')
            column_names = next(csv_reader)

            value_names = [column_names[i] for i in columns]
            values = [[] for _ in range(len(columns))]

            for row in csv_reader:
                for i in range(len(columns)):
                    values[i].append(float(row[columns[i]]))

    except IOError:
        print('File \'{}\' could not be opened; skipping file'.format(filename))
        return [], [], [], []
    
    return values

def highestTimePerSegment(data, names):
    """Find the time spend in each segment within an experiment and get the highest value among all the experiments
    This value can be highest of either
    1. time between first planning(planning_results0.csv) and last planning(planning_results0.csv) in the same segment or
    2. time between first planning(planning_results0.csv) and last execution(planner_results_ec0.csv) in the same segment"""
    print("******************Highest time taken in a segment****************")
    print("\t", end=' ')
    for l in range(len(data)):
        print(names[l], end=' ')
    print()
    print("\t", end=' ')
    for l in range(len(data)):
        values = []

        for i in range(len(data[l]['times2'])):
            seg_counter1 = Counter(data[l]['segment1'][i])
            seg_counter2 = Counter(data[l]['segment2'][i])
            sorted_seg_counter1 = dict(sorted(seg_counter1.items()))
            sorted_seg_counter2 = dict(sorted(seg_counter2.items()))
            curr_index1 = 0
            curr_index2 = 0
            for r in range(len(sorted_seg_counter1)):
                values.append(data[l]['times1'][i][curr_index1+sorted_seg_counter1[r]-1] - data[l]['times2'][i][curr_index2])
                values.append(data[l]['times2'][i][curr_index2+sorted_seg_counter2[r]-1] - data[l]['times2'][i][curr_index2])
                curr_index1 += sorted_seg_counter1[r]
                curr_index2 += sorted_seg_counter2[r]

        print(max(values)+5, end=' ')# 5 for the final graph building time  
    print()
            
            

def highestPlanningNumPerSeg(data, names):
    """Highest number of plannings per experiment in any segment
    count the number of rows with the same segment number in planning_results0.csv file and find the largest count"""
    print("******************Highest number of planning in a segment****************")
    print("\t", end=' ')
    for l in range(len(data)):
        print(names[l], end=' ')
    print()
    print("\t", end=' ')
    for l in range(len(data)):
        values = []
        for i in range(len(data[l]['times2'])):
            seg_counter = Counter(data[l]['segment2'][i])
            values.append(max(seg_counter.values()))
        print(max(values), end=' ')
    print()

def highestExecutionNumPerSeg(data, names):
    """Highest number of executions per experiment in any segment
    count the number of rows with the same segment number in planner_results_ec0.csv file and find the largest count"""
    print("******************Highest number of executions in a segment****************")
    print("\t", end=' ')
    for l in range(len(data)):
        print(names[l], end=' ')
    print()
    print("\t", end=' ')
    for l in range(len(data)):
        values = []
        for i in range(len(data[l]['times1'])):
            seg_counter = Counter(data[l]['segment1'][i])
            values.append(max(seg_counter.values()))
        print(max(values), end=' ')
    print()

def averageExecutionNumPerExp(data, names):
    """average total number of executions per experiment
    checking the number of rows-1 in planner_results_ec0.csv file"""
    average_vals = []
    std_dev = []

    for l in range(len(data)):
        values = []
        for i in range(len(data[l]['times1'])):
            values.append(len(data[l]['times1'][i]))
        average_vals.append(statistics.mean(values))
        std_dev.append(statistics.stdev(values))

    print("******************average number of executions per experiment****************")
    print("\t", end=' ')
    for l in range(len(data)):
        print(names[l], end=' ')
    print()
    print("averageExecutionNumPerExp-mean", end=' ')
    for l in range(len(data)):
        print(average_vals[l], end=' ')
    print()
    print("averageExecutionNumPerExp-std", end=' ')
    for l in range(len(data)):
        print(std_dev[l], end=' ')
    print()

def averageExecutionCostPerExp(data, names):
    """average of last row value for Plan length in planner_results_ec0.csv file"""
    average_vals = []
    std_dev = []

    for l in range(len(data)):
        values = []
        for i in range(len(data[l]['times1'])):
            values.append(data[l]['planLength'][i][-1])
        average_vals.append(statistics.mean(values))
        std_dev.append(statistics.stdev(values))

    print("******************average execution cost per experiment****************")
    print("\t", end=' ')
    for l in range(len(data)):
        print(names[l], end=' ')
    print()
    print("averageExecutionCostPerExp-mean", end=' ')
    for l in range(len(data)):
        print(average_vals[l], end=' ')
    print()
    print("averageExecutionCostPerExp-std", end=' ')
    for l in range(len(data)):
        print(std_dev[l], end=' ')
    print()

def averageExecutionTimePerExp(data, names):
    """average of last row value for Plan duration (s) in planner_results_ec0.csv file"""
    average_vals = []
    std_dev = []

    for l in range(len(data)):
        values = []
        for i in range(len(data[l]['times1'])):
            values.append(data[l]['planDuration'][i][-1])
        average_vals.append(statistics.mean(values))
        std_dev.append(statistics.stdev(values))

    print("******************average execution time per experiment****************")
    print("\t", end=' ')
    for l in range(len(data)):
        print(names[l], end=' ')
    print()
    print("averageExecutionTimePerExp-mean", end=' ')
    for l in range(len(data)):
        print(average_vals[l], end=' ')
    print()
    print("averageExecutionTimePerExp-std", end=' ')
    for l in range(len(data)):
        print(std_dev[l], end=' ')
    print()

# def highestOptimizationFaliureCountPerExp(data, names):
#     """
#     If no execution between two successive optimizations in the same segment
#     """
#     print("******************highest optimization failure count per experiment****************")
#     print("\t", end=' ')
#     for l in range(len(data)):
#         print(names[l], end=' ')
#     print()
#     print("\t", end=' ')
#     for l in range(len(data)):
#         faliures = []
#         for i in range(len(data[l]['times2'])):
#             faliure = 0
#             seg_counter1 = Counter(data[l]['segment1'][i])
#             seg_counter2 = Counter(data[l]['segment2'][i])
#             sorted_seg_counter1 = dict(sorted(seg_counter1.items()))

#             dict_values = list(sorted_seg_counter1.values())
#             cumulative_sums = []
#             curr_sum = 0
#             for val in dict_values:
#                 curr_sum += val
#                 cumulative_sums.append(curr_sum)

#             sorted_seg_counter2 = dict(sorted(seg_counter2.items()))

#             curr_index1 = 0
#             curr_index2 = 0

#             for s in range(len(sorted_seg_counter2)): # 16 times - segment count
#                 for e in range(sorted_seg_counter2[s]): # optimization number of times
#                     success = False
#                     for r in range(curr_index1, cumulative_sums[s]): # execution number of times
#                         if (curr_index2+e+1 >= len(data[l]['times2'][i])):
#                             if (data[l]['times1'][i][r]>data[l]['times2'][i][curr_index2+e]):
#                                 success = True
#                                 continue
#                             else:
#                                 if (success == False):
#                                     faliure +=1
#                                 print("detected faliure")
#                                 print("while comparing " + str(r+2) + " to " + str(curr_index2+e+2) + " and " + str(curr_index2+e+1+2))
#                             break


#                         if (data[l]['times1'][i][r]>data[l]['times2'][i][curr_index2+e] and data[l]['times1'][i][r]<data[l]['times2'][i][curr_index2+e+1]):
#                             success = True
#                             curr_index1 += 1
#                             continue
#                         else:
#                             if (success == False):
#                                 faliure +=1
#                                 print("detected faliure")
#                                 print("while comparing " + str(r+2) + " to " + str(curr_index2+e+2) + " and " + str(curr_index2+e+1+2))
#                             break
#                 curr_index2 += sorted_seg_counter2[s]

#             faliures.append(faliure)

#         print(faliures)
#         # print(max(faliures), end=' ')
#     print()
#     sys.exit(0)



# def averageTimeInSegPerExp(data, names):
#     """
#     Average time spend within an experiment within a segment
#     """
#     print("******************Highest time taken in a segment****************")
#     print("\t", end=' ')
#     for l in range(len(data)):
#         print(names[l], end=' ')
#     print()
#     print("\t", end=' ')
#     for l in range(len(data)):
#         values1 = []
#         for i in range(len(data[l]['times2'])):
#             values2 = []
#             seg_counter = Counter(data[l]['segment2'][i])
#             sorted_seg_counter = dict(sorted(seg_counter.items()))
#             curr_index = 0
#             print(sorted_seg_counter)
#             for _, count in sorted_seg_counter.items():
#                 values2.append(data[l]['times2'][i][curr_index+count-1] - data[l]['times2'][i][curr_index])
#                 curr_index += count
#             print(values2)
#             sys.exit(0)




def evaluate(data, names):
    # highestOptimizationFaliureCountPerExp(data, names)
    highestTimePerSegment(data, names)
    highestPlanningNumPerSeg(data, names)
    highestExecutionNumPerSeg(data, names)
    averageExecutionNumPerExp(data, names)
    averageExecutionCostPerExp(data, names)
    averageExecutionTimePerExp(data, names)



method1_folders = ["/home/thesis/graphopt_results/exp9/1/1.1/",
                   "/home/thesis/graphopt_results/exp9/1/1.2/",
                   "/home/thesis/graphopt_results/exp9/1/1.3/",
                   "/home/thesis/graphopt_results/exp9/1/1.4/",
                   "/home/thesis/graphopt_results/exp9/1/1.5/"]
method2_folders = ["/home/thesis/graphopt_results/exp9/2/2.1/",
                   "/home/thesis/graphopt_results/exp9/2/2.2/",
                   "/home/thesis/graphopt_results/exp9/2/2.3/",
                   "/home/thesis/graphopt_results/exp9/2/2.4/",
                   "/home/thesis/graphopt_results/exp9/2/2.5/"]
method3_folders = ["/home/thesis/graphopt_results/exp9/3/3.1/",
                   "/home/thesis/graphopt_results/exp9/3/3.2/",
                   "/home/thesis/graphopt_results/exp9/3/3.3/",
                   "/home/thesis/graphopt_results/exp9/3/3.4/",
                   "/home/thesis/graphopt_results/exp9/3/3.5/"]


file1_method1 = [method1_folder + 'planner_results_ec0.csv' for method1_folder in method1_folders]
file1_method2 = [method2_folder + 'planner_results_ec0.csv' for method2_folder in method2_folders]
file1_method3 = [method3_folder + 'planner_results_ec0.csv' for method3_folder in method3_folders]
# file1_method4 = [method4_folder + 'planner_results_ec0.csv' for method4_folder in method4_folders]
# file1_method5 = [method5_folder + 'planner_results_ec0.csv' for method5_folder in method5_folders]

file2_method1 = [method1_folder + 'planning_results0.csv' for method1_folder in method1_folders]
file2_method2 = [method2_folder + 'planning_results0.csv' for method2_folder in method2_folders]
file2_method3 = [method3_folder + 'planning_results0.csv' for method3_folder in method3_folders]
# file2_method4 = [method4_folder + 'planning_results0.csv' for method4_folder in method4_folders]
# file2_method5 = [method5_folder + 'planning_results0.csv' for method5_folder in method5_folders]


data_method1 = {'times1': [], 'planDuration': [], 'planLength': [], 'segment1': [], 'times2': [], 'segment2': []}
data_method2 = {'times1': [], 'planDuration': [], 'planLength': [], 'segment1': [], 'times2': [], 'segment2': []}
data_method3 = {'times1': [], 'planDuration': [], 'planLength': [], 'segment1': [], 'times2': [], 'segment2': []}
# data_method4 = {'times1': [], 'planDuration': [], 'planLength': [], 'segment1': [], 'times2': [], 'segment2': []}
# data_method5 = {'times1': [], 'planDuration': [], 'planLength': [], 'segment1': [], 'times2': [], 'segment2': []}


columns = [0, 1, 2, 22]

# extract the times, values etc from each file as a list of lists
for i in range(len(method1_folders)):
    values1 = readValues1(file1_method1[i], columns)
    values2 = readValues2(file2_method1[i], columns[:2])
    data_method1['times1'].append(values1[0])
    data_method1['planDuration'].append(values1[1])
    data_method1['planLength'].append(values1[2])
    data_method1['segment1'].append(values1[3])
    data_method1['times2'].append(values2[0])
    data_method1['segment2'].append(values2[1])
for i in range(len(method2_folders)):
    values1 = readValues1(file1_method2[i], columns)
    values2 = readValues2(file2_method2[i], columns[:2])
    data_method2['times1'].append(values1[0])
    data_method2['planDuration'].append(values1[1])
    data_method2['planLength'].append(values1[2])
    data_method2['segment1'].append(values1[3])
    data_method2['times2'].append(values2[0])
    data_method2['segment2'].append(values2[1])
for i in range(len(method3_folders)):
    values1 = readValues1(file1_method3[i], columns)
    values2 = readValues2(file2_method3[i], columns[:2])
    data_method3['times1'].append(values1[0])
    data_method3['planDuration'].append(values1[1])
    data_method3['planLength'].append(values1[2])
    data_method3['segment1'].append(values1[3])
    data_method3['times2'].append(values2[0])
    data_method3['segment2'].append(values2[1])
# for i in range(len(method4_folders)):
#     values1 = readValues1(file1_method4[i], columns)
#     values2 = readValues2(file2_method4[i], columns[:2])
#     data_method4['times1'].append(values1[0])
#     data_method4['planDuration'].append(values1[1])
#     data_method4['planLength'].append(values1[2])
#     data_method4['segment1'].append(values1[3])
#     data_method4['times2'].append(values2[0])
#     data_method4['segment2'].append(values2[1])
# for i in range(len(method5_folders)):
#     values1 = readValues1(file1_method5[i], columns)
#     values2 = readValues2(file2_method5[i], columns[:2])
#     data_method5['times1'].append(values1[0])
#     data_method5['planDuration'].append(values1[1])
#     data_method5['planLength'].append(values1[2])
#     data_method5['segment1'].append(values1[3])
#     data_method5['times2'].append(values2[0])
#     data_method5['segment2'].append(values2[1])

plotnames = ["Random", "High", "Low"]


evaluate([data_method1, data_method2, data_method3], plotnames)