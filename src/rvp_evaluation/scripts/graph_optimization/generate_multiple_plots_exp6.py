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

NORM_TO_ONE_SET = {'Volume accuracy', 'Covered ROI volume',
                   'Vol. acc.', 'Vol. acc. bbx',
                   'Vol. acc. ma0', 'Vol. acc. bbx ma0',
                   'Vol. acc. ma50', 'Vol. acc. bbx ma50'}
NORM_TO_ONE_HALF_SET = {'Ratio', 'Ratio bbx',
                        'Ratio ma0', 'Ratio bbx ma0',
                        'Ratio ma50', 'Ratio bbx ma50'}
NORM_TO_DIST_SET = {'Dist.', 'Centroid error(mm)', 'Centroid error(vol>50%)(mm)'}
NORM_TO_FRUIT_NUM_SET = {'Detected ROI cluster', 'Fruits detected', 'Fruits detected (vol>50%)'}
NORM_TO_VOL_SET = {'Vol. acc.', 'Volume accuracy', 'Volume accuracy (vol>50%)'}
#TITLE_MAP = {'Average volume accuracy': 'Volume accuracy', 'Det. cluster ma50': 'Detected fruits', 'Dist. ma50': 'Center distance', 'Vol. acc. ma50': 'Volume accuracy'}
TITLE_MAP = {'Det. cluster ma0': 'Fruits detected', 'Det. cluster ma50': 'Fruits detected (vol>50%)', 'Dist. ma0': 'Centroid error(mm)', 'Dist. ma50': 'Centroid error(vol>50%)(mm)',
             'Vol. acc. ma0': 'Volume accuracy', 'Vol. acc. ma50': 'Volume accuracy (vol>50%)'}
FRUIT_NUM = 100
#COLORS = ['darkblue', 'blue', 'lightblue', 'firebrick', 'red', 'salmon']
#COLORS = ['darkblue', 'firebrick', 'green', 'lightblue', 'magenta', 'yellow', 'salmon']
COLORS = ['darkblue', 'firebrick', 'forestgreen', 'magenta', 'darkcyan']

# the number of plots = number of combinations of x-labels and y-labels
# in this case, x_label = len(time_columns) = 1
# y_labels = len(columns_ec) = 10
# so 10 plots

def generatemultiplePlots(dataset, out_folder, plotnames):
    for i in range(len(dataset[0]['time_names'][0])): # x -axis

        # find the highest time
        xmax = 0
        for d in range(len(dataset)):
            if dataset[d]['times'][:][i][-1][-1]>xmax:
                xmax = dataset[d]['times'][:][i][-1][-1]

        xmax_int = int(np.ceil(xmax)) # make it int
        times_inter = np.linspace(0, xmax_int, xmax_int + 1, dtype=int) # 0 -> max time at 1 second accuracy



        # value_name = ['Detected ROI cluster', 'Dist.', 'Vol. acc.', 'Det. cluster ma0', 'Dist. ma0', 'Vol. acc. ma0', 'Det. cluster ma50', 'Dist. ma50', 'Vol. acc. ma50', 'Segment']
        for j in range(len(dataset[0]['value_names'][0])): # plotting each plot
            if (dataset[0]['value_names'][0][j] in TITLE_MAP): # take the mapped value if it is in the dictionary
                dataset[0]['value_names'][0][j] = TITLE_MAP[dataset[0]['value_names'][0][j]]

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


            # plot vertical line at the end of the beginning of each segment. beginning of each segment is already same for all
            for s in dataset[0]['segment_starts'][0]:
                if s > 0 and s < len(dataset[0]['times'][0][0]):
                    svlines = dataset[0]['times'][0][0][s]
                    plt.vlines(svlines, 0, 1, transform=plt.gca().get_xaxis_transform())


            # Below operation is the interpolation operation
            # we are looking at the value indicated by j which is going to be plotted
            # take 5 experiments of each methof, using each expriment's time stamp and the corrresponding value, it outputs a modified interpolated values at uniform time interval of 1 second.
            values_inter = [[] for _ in range(len(dataset))]

            # len(times_rvp) = number of experiments from each. in our case, 5 of rvp and 5 of vmp. so 5
            for t in range(len(dataset[0]['times'])):
                for l in range(len(dataset)):
                    values_inter[l].append(np.interp(times_inter, dataset[l]['times'][t][i], dataset[l]['values'][t][j]))

            # each row is each experiment
            # each column is the value at 1 seconds time interval
            # taking mean at each timestamp
            # taking std at each timestamp
            values_mean = [np.mean(li, 0) for li in values_inter]
            values_std = [np.std(li, 0) for li in values_inter]

            if (dataset[0]['value_names'][0][j] in NORM_TO_DIST_SET):
                values_mean = [arr * 1000 for arr in values_mean]
                values_std = [arr * 1000 for arr in values_std]

            # times_inter -> x axis timestamp
            # values_rvp_mean -> the value at each timestamp
            # label to be used
            for l in range(len(dataset)):
                plt.plot(times_inter, values_mean[l], linewidth=2.0, label=plotnames[l], color=COLORS[l])
                # fill_between is used to fill the standard deviation region
                # plt.fill_between(times_inter, values_mean[l]-values_std[l], values_mean[l]+values_std[l], alpha=0.5)

            plt.xlabel(dataset[0]['time_names'][0][i]) # the header name in the csv file for the time
            plt.ylabel(dataset[0]['value_names'][0][j]) # the header name in the csv file for the value

            plt.xlim(0, xmax) # set x limit
            if (dataset[0]['value_names'][0][j] in NORM_TO_FRUIT_NUM_SET): # for plots with the cluster number, use the FRUIT_NUM as the y limit
                plt.ylim((0, FRUIT_NUM))
            elif (dataset[0]['value_names'][0][j] in NORM_TO_DIST_SET):
                plt.ylim((0, 20))
            elif (dataset[0]['value_names'][0][j] in NORM_TO_VOL_SET):
                plt.ylim((0.5, 1))
            #elif (value_names[j] in NORM_TO_ONE_SET):
            #    plt.gca().yaxis.set_major_formatter(mtick.PercentFormatter(1.0))
            #    plt.ylim((0, 1))
            # plt.title(value_names[j])
            plt.legend(ncol=2)
            plt.grid()
            plt.tight_layout()
            figname =  dataset[0]['value_names'][0][j].lower().replace(" ", "_") + "_" + dataset[0]['time_names'][0][i].lower().replace(" ", "_") + ".png" # file name
            plt.savefig(os.path.join(out_folder, figname))
            plt.clf()
            
            # for t in range(len(dataset[0]['times'])):
            #     print(figname, end=' ')
            #     for l in range(len(dataset)):
            #         print(dataset[l]['values'][t][j][-1], end=' ') # print the final value at the end of the experiment
            #     print()
            
            # print mean and std of all values
            print("Figure Name", end=' ')
            for l in range(len(dataset)):
                print(plotnames[l], end=' ')
            print()
            print(figname+"-mean", end=' ')
            for l in range(len(dataset)):
                print(values_mean[l][-1], end=' ')
            print()
            print(figname+"-std", end=' ')
            for l in range(len(dataset)):
                print(values_std[l][-1], end=' ')
            print()


def multipleMoveSegmentStartTimes(times, segment_starts):
    # times - a list which contain times_method1, times_method2, times_method3, times_method4, times_method5
    # similarly segment_starts

    # to ensure the number of segments are same in the both experiment
    seg_count = len(segment_starts[0])
    if all(len(sublist) == seg_count for sublist in segment_starts):
        pass
    else:
        print('Invalid segment starts')
        return
    
    # iterate through each segment
    for i in range(seg_count - 1):
        seg = [s[i] for s in segment_starts]
        seg_next = [s[i+1] for s in segment_starts]

        # the faster one should be slowed down by using offset
        # identify which is later among all
        latest_time = float('-inf')  # Initialize with the lowest possible value
        latest_time_index = None
        for j in range(len(times)):
            if latest_time<times[j][0][seg[j]]:
                latest_time=times[j][0][seg[j]]
                latest_time_index=j
        # add offset to all
        for j in range(len(times)):
            vmp_offset = latest_time - times[j][0][seg[j]]
            for k in range(seg[j], seg_next[j]):
                times[j][0][k] += vmp_offset
                

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

time_columns = [0]
#columns_ec = [3, 4, 5, 9, 10, 11, 15, 16, 17, 22]
columns_ec = [z for z in range(1,21)]
columns_ec = columns_ec + [22]



method1_folders = ["/home/thesis/graphopt_results/exp6/1/1.1/",
                   "/home/thesis/graphopt_results/exp6/1/1.2/",
                   "/home/thesis/graphopt_results/exp6/1/1.3/",
                   "/home/thesis/graphopt_results/exp6/1/1.4/",
                   "/home/thesis/graphopt_results/exp6/1/1.5/"]
method2_folders = ["/home/thesis/graphopt_results/exp6/2/2.1/",
                   "/home/thesis/graphopt_results/exp6/2/2.2/",
                   "/home/thesis/graphopt_results/exp6/2/2.3/",
                   "/home/thesis/graphopt_results/exp6/2/2.4/",
                   "/home/thesis/graphopt_results/exp6/2/2.5/"]
method3_folders = ["/home/thesis/graphopt_results/exp6/3/3.1/",
                   "/home/thesis/graphopt_results/exp6/3/3.2/",
                   "/home/thesis/graphopt_results/exp6/3/3.3/",
                   "/home/thesis/graphopt_results/exp6/3/3.4/",
                   "/home/thesis/graphopt_results/exp6/3/3.5/"]
method4_folders = ["/home/thesis/graphopt_results/exp6/4/4.1/",
                   "/home/thesis/graphopt_results/exp6/4/4.2/",
                   "/home/thesis/graphopt_results/exp6/4/4.3/",
                   "/home/thesis/graphopt_results/exp6/4/4.4/",
                   "/home/thesis/graphopt_results/exp6/4/4.5/"]
method5_folders = ["/home/thesis/graphopt_results/exp6/5/5.1/",
                   "/home/thesis/graphopt_results/exp6/5/5.2/",
                   "/home/thesis/graphopt_results/exp6/5/5.3/",
                   "/home/thesis/graphopt_results/exp6/5/5.4/",
                   "/home/thesis/graphopt_results/exp6/5/5.5/"]

file_method1 = [method1_folder + 'planner_results_ec0.csv' for method1_folder in method1_folders]
file_method2 = [method2_folder + 'planner_results_ec0.csv' for method2_folder in method2_folders]
file_method3 = [method3_folder + 'planner_results_ec0.csv' for method3_folder in method3_folders]
file_method4 = [method4_folder + 'planner_results_ec0.csv' for method4_folder in method4_folders]
file_method5 = [method5_folder + 'planner_results_ec0.csv' for method5_folder in method5_folders]

data_method1 = {'times': [], 'values': [], 'time_names': [], 'value_names': [], 'segment_starts': [], 'segments': []}
data_method2 = {'times': [], 'values': [], 'time_names': [], 'value_names': [], 'segment_starts': [], 'segments': []}
data_method3 = {'times': [], 'values': [], 'time_names': [], 'value_names': [], 'segment_starts': [], 'segments': []}
data_method4 = {'times': [], 'values': [], 'time_names': [], 'value_names': [], 'segment_starts': [], 'segments': []}
data_method5 = {'times': [], 'values': [], 'time_names': [], 'value_names': [], 'segment_starts': [], 'segments': []}


# extract the times, values etc from each file as a list of lists
for i in range(len(method1_folders)):
    t, v, tn, vn, ss = readValues(file_method1[i], time_columns, columns_ec, seg_column=22)
    data_method1['times'].append(t) # [[[9.423, 19.574, 23.011]], [[3.63, 12.44, 18.01]], ....]
    data_method1['values'].append(v)
    data_method1['time_names'].append(tn)
    data_method1['value_names'].append(vn)
    data_method1['segment_starts'].append(ss)
    data_method1['segments'].append(v[-1]) # [0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, ...] -> segment numbers
for i in range(len(method2_folders)):
    t, v, tn, vn, ss = readValues(file_method2[i], time_columns, columns_ec, seg_column=22)
    data_method2['times'].append(t) # [[[9.423, 19.574, 23.011]], [[3.63, 12.44, 18.01]], ....]
    data_method2['values'].append(v)
    data_method2['time_names'].append(tn)
    data_method2['value_names'].append(vn)
    data_method2['segment_starts'].append(ss)
    data_method2['segments'].append(v[-1]) # [0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, ...] -> segment numbers
for i in range(len(method3_folders)):
    t, v, tn, vn, ss = readValues(file_method3[i], time_columns, columns_ec, seg_column=22)
    data_method3['times'].append(t) # [[[9.423, 19.574, 23.011]], [[3.63, 12.44, 18.01]], ....]
    data_method3['values'].append(v)
    data_method3['time_names'].append(tn)
    data_method3['value_names'].append(vn)
    data_method3['segment_starts'].append(ss)
    data_method3['segments'].append(v[-1]) # [0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, ...] -> segment numbers
for i in range(len(method4_folders)):
    t, v, tn, vn, ss = readValues(file_method4[i], time_columns, columns_ec, seg_column=22)
    data_method4['times'].append(t) # [[[9.423, 19.574, 23.011]], [[3.63, 12.44, 18.01]], ....]
    data_method4['values'].append(v)
    data_method4['time_names'].append(tn)
    data_method4['value_names'].append(vn)
    data_method4['segment_starts'].append(ss)
    data_method4['segments'].append(v[-1]) # [0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, ...] -> segment numbers
for i in range(len(method5_folders)):
    t, v, tn, vn, ss = readValues(file_method5[i], time_columns, columns_ec, seg_column=22)
    data_method5['times'].append(t) # [[[9.423, 19.574, 23.011]], [[3.63, 12.44, 18.01]], ....]
    data_method5['values'].append(v)
    data_method5['time_names'].append(tn)
    data_method5['value_names'].append(vn)
    data_method5['segment_starts'].append(ss)
    data_method5['segments'].append(v[-1]) # [0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, ...] -> segment numbers


# the below operation is performed to make sure that the time at start of an segment in both the experiments are the same. the end time of each segment may vary as one experiment may have more number of trajectories than the other. so the one with less trajectories have a horizontal line seen in the plot during this time.

# adjusting the time of experiments among themselves to have the same time in each segment
for i in range(len(method1_folders) - 1):
    moveSegmentStartTimes(data_method1['times'][i], data_method1['times'][i+1], data_method1['segment_starts'][i], data_method1['segment_starts'][i+1])
for i in range(len(method2_folders) - 1):
    moveSegmentStartTimes(data_method2['times'][i], data_method2['times'][i+1], data_method2['segment_starts'][i], data_method2['segment_starts'][i+1])
for i in range(len(method3_folders) - 1):
    moveSegmentStartTimes(data_method3['times'][i], data_method3['times'][i+1], data_method3['segment_starts'][i], data_method3['segment_starts'][i+1])
for i in range(len(method4_folders) - 1):
    moveSegmentStartTimes(data_method4['times'][i], data_method4['times'][i+1], data_method4['segment_starts'][i], data_method4['segment_starts'][i+1])
for i in range(len(method5_folders) - 1):
    moveSegmentStartTimes(data_method5['times'][i], data_method5['times'][i+1], data_method5['segment_starts'][i], data_method5['segment_starts'][i+1])

# adjusting the time of rvp and vmp experiments
for i in range(len(method1_folders)):
    multipleMoveSegmentStartTimes([data_method1['times'][i], data_method2['times'][i], data_method3['times'][i], data_method4['times'][i], data_method5['times'][i]],
                                  [data_method1['segment_starts'][i], data_method2['segment_starts'][i], data_method3['segment_starts'][i], data_method4['segment_starts'][i], data_method5['segment_starts'][i]])



plotnames = ["20v + cutoff", "10v + cutoff", "5v + cutoff", "10v", "5v"]
# time_names_rvp[0] -> time column names
# value_names_rvp[0] -> value column names
generatemultiplePlots([data_method1, data_method2, data_method3, data_method4, data_method5], out_folder, plotnames)