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

def readValues(filename, time_columns=[], value_columns=[], seg_column=22):
    times = []
    values = []
    segment_starts = []
    try:
        with open(filename, 'r') as csvfile:
            csv_reader = csv.reader(csvfile, delimiter=',')
            column_names = next(csv_reader) # get the header of the csv file
            if not time_columns: # default time columns: first three
                time_columns=[0, 1, 2]
            if not value_columns:  # if no columns given, read all but time
                value_columns = range(len(time_columns), len(column_names))
            
            time_names = [column_names[i] for i in time_columns] # ['Time (s)']
            value_names = [column_names[i] for i in value_columns] # ['Detected ROI cluster', 'Dist.', 'Vol. acc.', 'Det. cluster ma0', 'Dist. ma0', 'Vol. acc. ma0', 'Det. cluster ma50', 'Dist. ma50', 'Vol. acc. ma50', 'Segment']

            times = [[] for _ in range(len(time_columns))]  # colums to read
            values = [[] for _ in range(len(value_columns))]  # colums to read
            cur_seg = -1
            index = 0
            for row in csv_reader:
                for i in range(len(time_columns)):
                    times[i].append(float(row[time_columns[i]]))
                for i in range(len(value_columns)):
                    values[i].append(float(row[value_columns[i]]))
                    
                seg = int(row[seg_column])
                while seg > cur_seg:
                    segment_starts.append(index)
                    cur_seg += 1
                
                index += 1
            
            # in other words - column extraction into lists from the csv file
            # after 3 iterations
            # times = [[9.423, 19.574, 23.011]] -> the time column value in each row
            # values = [[3.0, 3.0, 3.0], [0.00636252, 0.00639121, 0.00628908], [0.827206, 0.835082, 0.838701], [3.0, 3.0, 3.0], [0.00636252, 0.00639121, 0.00628908], [0.827206, 0.835082, 0.838701], [3.0, 3.0, 3.0], [0.00636252, 0.00639121, 0.00628908], [0.827206, 0.835082, 0.838701], [0.0, 0.0, 0.0]] -> each element [] consits of same column value
            # seg is always 0.
            # segment_starts = [0] -> value wont change after first iteration
                
            segment_starts.append(index)
            # segment_starts = [0, 9, 18, 24, 29, 35, 40, 46, 52, 57, 62, 66, 70, 75, 79, 82, 86] -> segment 0 starts at row 0, segment 1 starts at row 9, segment 2 starts at row 18 -> can compute have trajectories executed per segment.

    except IOError:
        print('File \'{}\' could not be opened; skipping file'.format(filename))
        return [], [], [], []

    return times, values, time_names, value_names, segment_starts

NORM_TO_ONE_SET = {'Volume accuracy', 'Covered ROI volume', 'Vol. acc.', 'Vol. acc. bbx', 'Vol. acc. ma0', 'Vol. acc. ma50'}
NORM_TO_DIST_SET = {'Dist.', 'Dist. ma0', 'Dist. ma50'}
NORM_TO_FRUIT_NUM_SET = {'Detected ROI cluster', 'Det. cluster ma0', 'Det. cluster ma50'}
TITLE_MAP = {'Average volume accuracy': 'Volume accuracy', 'Det. cluster ma50': 'Detected fruits', 'Dist. ma50': 'Center distance',
'Vol. acc. ma50': 'Volume accuracy'}
FRUIT_NUM = 100
#COLORS = ['darkblue', 'blue', 'lightblue', 'firebrick', 'red', 'salmon']
COLORS = ['darkblue', 'lightblue', 'firebrick', 'salmon']

# the number of plots = number of combinations of x-labels and y-labels
# in this case, x_label = len(time_columns) = 1
# y_labels = len(columns_ec) = 10
# so 10 plots

def generatePlots(time_names, value_names, times_rvp, values_rvp, segments_rvp, segment_starts_rvp, times_vmp, values_vmp, segments_vmp, segment_starts_vmp, out_folder):
    for i in range(len(time_names)): # x -axis
        # i is always 0
        # so among all lists, look at last element which is the final time. take the max of all experiments
        xmax = max(times_rvp[:][i][-1] + times_vmp[:][i][-1])
        xmax_int = int(np.ceil(xmax)) # make it int
        times_inter = np.linspace(0, xmax_int, xmax_int + 1, dtype=int) # 0 -> max time at 1 second accuracy
        
        # value_name = ['Detected ROI cluster', 'Dist.', 'Vol. acc.', 'Det. cluster ma0', 'Dist. ma0', 'Vol. acc. ma0', 'Det. cluster ma50', 'Dist. ma50', 'Vol. acc. ma50', 'Segment']
        for j in range(len(value_names)): # plotting each plot
            if (value_names[i] in TITLE_MAP): # take the mapped value if it is in the dictionary
                value_names[i] = TITLE_MAP[value_names[i]]

            fig = plt.figure(i, figsize=figaspect(1))

            

            FONT_SIZE = 14

            plt.rc('font', size=FONT_SIZE)          # controls default text sizes
            plt.rc('axes', titlesize=FONT_SIZE)     # fontsize of the axes title
            plt.rc('axes', labelsize=FONT_SIZE)    # fontsize of the x and y labels
            plt.rc('xtick', labelsize=FONT_SIZE)    # fontsize of the tick labels
            plt.rc('ytick', labelsize=FONT_SIZE)    # fontsize of the tick labels
            plt.rc('legend', fontsize=10)    # legend fontsize
            plt.rc('figure', titlesize=FONT_SIZE)  # fontsize of the figure title

            vals = np.linspace(0,1,256)
            np.random.shuffle(vals)
            cmap = plt.cm.colors.ListedColormap(plt.cm.jet(vals)) # color map

            # segment_starts_vmp = [[0, 8, 15, 20, 26, 30, 35, 40, 45, 48, 52, 56, 59, 63, 66, 71, 75], [0, 8, 15, 20, 26, 30, 35, 40, 45, 48, 52, 56, 59, 63, 66, 71, 75]]
            # 0   75
            # 8   75
            # 15   75
            # 20   75
            # 26   75
            # 30   75
            # 35   75
            # 40   75
            # 45   75
            # 48   75
            # 52   75
            # 56   75
            # 59   75
            # 63   75
            # 66   75
            # 71   75
            # 75   75
            # plot vertical line at the end of the beginning of each segment. beginning of each segment is already same for both RVP and VMP
            for s in segment_starts_vmp[0]:
                if s > 0 and s < len(times_vmp[0][0]):
                    svlines = times_vmp[0][0][s]
                    plt.vlines(svlines, 0, 1, transform=plt.gca().get_xaxis_transform())


            # Below operation is the interpolation operation
            # we are looking at the value indicated by j which is going to be plotted
            # take 5 experiments of rvp, using each expriment's time stamp and the corrresponding value, it outputs a modified interpolated values at uniform time interval of 1 second.
            values_rvp_inter = []
            values_vmp_inter = []
            # len(times_rvp) = number of experiments from each. in our case, 5 of rvp and 5 of vmp. so 5
            for t in range(len(times_rvp)):
                values_rvp_inter.append(np.interp(times_inter, times_rvp[t][i], values_rvp[t][j]))
                values_vmp_inter.append(np.interp(times_inter, times_vmp[t][i], values_vmp[t][j]))
            # each row is each experiment
            # each column is the value at 1 seconds time interval
            # taking mean at each timestamp
            # taking std at each timestamp
            values_rvp_mean = np.mean(values_rvp_inter, 0)
            values_rvp_std = np.std(values_rvp_inter, 0)
            values_vmp_mean = np.mean(values_vmp_inter, 0)
            values_vmp_std = np.std(values_vmp_inter, 0)

            # times_inter -> x axis timestamp
            # values_rvp_mean -> the value at each timestamp
            # label to be used
            plt.plot(times_inter, values_rvp_mean, linewidth=2.0, label="VMP", color='darkblue')
            # fill_between is used to fill the standard deviation region
            plt.fill_between(times_inter, values_rvp_mean-values_rvp_std, values_rvp_mean+values_rvp_std, alpha=0.5)
            plt.plot(times_inter, values_vmp_mean, linewidth=2.0, label="VMP-graphOpt", color='firebrick')
            plt.fill_between(times_inter, values_vmp_mean-values_vmp_std, values_vmp_mean+values_vmp_std, alpha=0.5)

            plt.xlabel(time_names[i]) # the header name in the csv file for the time
            plt.ylabel(value_names[j]) # the header name in the csv file for the value

            plt.xlim(0, xmax) # set x limit
            if (value_names[j] in NORM_TO_FRUIT_NUM_SET): # for plots with the cluster number, use the FRUIT_NUM as the y limit
                plt.ylim((0, FRUIT_NUM))
            elif (value_names[j] in NORM_TO_DIST_SET):
                plt.ylim((0, 0.05))
            #elif (value_names[j] in NORM_TO_ONE_SET):
            #    plt.gca().yaxis.set_major_formatter(mtick.PercentFormatter(1.0))
            #    plt.ylim((0, 1))
            # plt.title(value_names[j])
            plt.legend(ncol=2)
            plt.grid()
            plt.tight_layout()
            figname =  value_names[j].lower().replace(" ", "_") + "_" + time_names[i].lower().replace(" ", "_") + ".png" # file name
            plt.savefig(os.path.join(out_folder, figname))
            plt.clf()
            
            for t in range(len(times_rvp)):
                print(figname, values_rvp[t][j][-1], values_vmp[t][j][-1]) # print the final value at the end of the experiment

def moveSegmentStartTimes(times_rvp, times_vmp, segment_starts_rvp, segment_starts_vmp):
    # to ensure the number of segments are same in the both experiment
    if len(segment_starts_rvp) != len(segment_starts_vmp):
        print('Invalid segment starts')
        return

    for i in range(len(segment_starts_rvp) - 1):
        # below are integers indicating the starting row number of the current segment and the next segment
        seg_rvp = segment_starts_rvp[i] 
        seg_rvp_next = segment_starts_rvp[i+1]
        seg_vmp = segment_starts_vmp[i]
        seg_vmp_next = segment_starts_vmp[i+1]
        
        # which ever experiment (vmp or rvp) is having the higher time value(starts a segment late - slower), for the other faster one, a offset can be computed as the difference between the times
        # the time value for the segment of the faster one can be added with this time offset to make it matching with the slower one
        # times_rvp[0] as [[]] structure
        if times_rvp[0][seg_rvp] > times_vmp[0][seg_vmp]:
            vmp_offset = times_rvp[0][seg_rvp] - times_vmp[0][seg_vmp]
            for j in range(seg_vmp, seg_vmp_next):
                times_vmp[0][j] += vmp_offset
        elif times_vmp[0][seg_vmp] > times_rvp[0][seg_rvp]:
            rvp_offset = times_vmp[0][seg_vmp] - times_rvp[0][seg_rvp]
            for j in range(seg_rvp, seg_rvp_next):
                times_rvp[0][j] += rvp_offset
    #return times_rvp, times_vmp

# Parameters
out_folder = "/home/thesis/results/plots"
out_folder_old = "/home/thesis/results/plots_old"
input1_range = range(0, 1)
input2_range = range(0, 1)
labels = ['RVP (all)', 'RVP (MA50)', 'VMP (all)', 'VMP (MA50)']

time_columns = [0]
columns_ec = [3, 4, 5, 9, 10, 11, 15, 16, 17, 22]
#columns_ec_ma0 = [9, 10, 11]
#columns_ec_ma50 = [15, 16, 17]
#columns = [columns_ec, columns_ec_ma50]

columns_old = [3, 6, 7, 8]

#rvp_folder = 'rvp/t3_plan_length_15/'
#vmp_folder = 'vmp/t3-2_plan_length_15/'

# rvp_folders = ['/home/thesis/baseline_results/default1/']
# rvp_folders = ['/home/thesis/baseline_results/default1/','/home/thesis/baseline_results/default2/',
#                '/home/thesis/baseline_results/default3/','/home/thesis/baseline_results/default4/','/home/thesis/baseline_results/default5/']
# rvp_folders = ['/home/thesis/graphopt_results/exp1/4/4.1/','/home/thesis/graphopt_results/exp1/4/4.2/',
#                '/home/thesis/graphopt_results/exp1/4/4.3/','/home/thesis/graphopt_results/exp1/4/4.4/','/home/thesis/graphopt_results/exp1/4/4.5/']
rvp_folders = ['/home/thesis/graphopt_results/exp11/2/2.1/','/home/thesis/graphopt_results/exp11/2/2.2/',
               '/home/thesis/graphopt_results/exp11/2/2.3/','/home/thesis/graphopt_results/exp11/2/2.4/','/home/thesis/graphopt_results/exp11/2/2.5/']
vmp_folders = ['/home/thesis/graphopt_results/exp11/3/3.1/','/home/thesis/graphopt_results/exp11/3/3.2/',
               '/home/thesis/graphopt_results/exp11/3/3.3/','/home/thesis/graphopt_results/exp11/3/3.4/','/home/thesis/graphopt_results/exp11/3/3.5/']
#vmp_folders = ['/home/thesis/graphopt_results/config1/test7/']

#rvp_folders = ['rvp/t11_w23_t60/', 'rvp/t13_w23_t60/', 'rvp/t15_w23_t60/', 'rvp/t17_w23_t60/', 'rvp/t19_w23_t60/']
#vmp_folders = ['vmp/t11_w23_t60/', 'vmp/t13_w23_t60/', 'vmp/t15_w23_t60/', 'vmp/t17_w23_t60/', 'vmp/t19_w23_t60/']

file_rvp = [rvp_folder + 'planner_results_ec0.csv' for rvp_folder in rvp_folders]
file_vmp = [vmp_folder + 'planner_results_ec0.csv' for vmp_folder in vmp_folders]

times_rvp, values_rvp, time_names_rvp, value_names_rvp, segment_starts_rvp, segments_rvp = [], [], [], [], [], []
times_vmp, values_vmp, time_names_vmp, value_names_vmp, segment_starts_vmp, segments_vmp = [], [], [], [], [], []

# extract the times, values etc from each file as a list of lists
for i in range(len(rvp_folders)):
    t, v, tn, vn, ss = readValues(file_rvp[i], time_columns, columns_ec, seg_column=22)
    times_rvp.append(t) # [[[9.423, 19.574, 23.011]], [[3.63, 12.44, 18.01]], ....]
    values_rvp.append(v)
    time_names_rvp.append(tn)
    value_names_rvp.append(vn)
    segment_starts_rvp.append(ss)
    segments_rvp.append(v[-1]) # [0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, ...] -> segment numbers

for i in range(len(vmp_folders)):
    t, v, tn, vn, ss = readValues(file_vmp[i], time_columns, columns_ec, seg_column=22)
    times_vmp.append(t)
    values_vmp.append(v)
    time_names_vmp.append(tn)
    value_names_vmp.append(vn)
    segment_starts_vmp.append(ss)
    segments_vmp.append(v[-1])

# the below operation is performed to make sure that the time at start of an segment in both the experiments are the same. the end time of each segment may vary as one experiment may have more number of trajectories than the other. so the one with less trajectories have a horizontal line seen in the plot during this time.

# adjusting the time of rvp experiments among themselves to have the same time in each segment
for i in range(len(rvp_folders) - 1):
    # times_rvp of file 1 and times_rvp of file 2.
    # segment starts of file 1 and of file2.
    moveSegmentStartTimes(times_rvp[i], times_rvp[i+1], segment_starts_rvp[i], segment_starts_rvp[i+1])
    #t1, t2 = moveSegmentStartTimes(times_rvp[i], times_rvp[i+1], segment_starts_rvp[i], segment_starts_rvp[i+1])
    #times_rvp[i] = t1
    #times_rvp[i+1] = t2

# adjusting the time of vmp experiments among themselves to have the same time in each segment
for i in range(len(vmp_folders) - 1):
    moveSegmentStartTimes(times_vmp[i], times_vmp[i+1], segment_starts_vmp[i], segment_starts_vmp[i+1])
    #t1, t2 = moveSegmentStartTimes(times_vmp[i], times_vmp[i+1], segment_starts_vmp[i], segment_starts_vmp[i+1])
    #times_vmp[i] = t1
    #times_vmp[i+1] = t2

# adjusting the time of rvp and vmp experiments
for i in range(len(rvp_folders)):
    moveSegmentStartTimes(times_rvp[i], times_vmp[i], segment_starts_rvp[i], segment_starts_vmp[i])
    #trvp, tvmp = moveSegmentStartTimes(times_rvp[i], times_vmp[i], segment_starts_rvp[i], segment_starts_vmp[i])
    #times_rvp[i] = trvp
    #times_vmp[i] = tvmp

# time_names_rvp[0] -> time column names
# value_names_rvp[0] -> value column names
generatePlots(time_names_rvp[0], value_names_rvp[0], times_rvp, values_rvp, segments_rvp, segment_starts_rvp, times_vmp, values_vmp, segments_vmp, segment_starts_vmp, out_folder)
