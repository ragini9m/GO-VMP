import pandas as pd
from typing import List
import statistics


def get_num_views_executed(data: pd.DataFrame):
    # 1 extra view because of the final extra recording
    return data.shape[0]-1

def get_num_plan_exec(data: pd.DataFrame):
    return data.shape[0]

def get_num_plan_exec_success(data: pd.DataFrame):
    filtered_df = data.loc[data["Success"] == True]
    return filtered_df.shape[0]

def get_plan_time(data: pd.DataFrame):
    return data['Plan Duration(s)'].sum()

def get_plan_time_sucess(data: pd.DataFrame):
    filtered_df = data.loc[data["Success"] == True]
    return filtered_df['Plan Duration(s)'].sum()

def get_exec_time(data: pd.DataFrame):
    return data['Execution Duration(s)'].sum()

def get_exec_time_success(data: pd.DataFrame):
    filtered_df = data.loc[data["Success"] == True]
    return filtered_df['Execution Duration(s)'].sum()

def get_segment_time(data: pd.DataFrame):
    time_in_segment = []

    for i in range(16):
        seg_df = data.loc[data["Segment"] == i]
        time = 0
        for j in range(seg_df.shape[0]):
            time = time + seg_df.iloc[j, 1] + seg_df.iloc[j, 2]
        time_in_segment.append(time)

    return time_in_segment

def get_segment_time_success(data: pd.DataFrame):
    filtered_df = data.loc[data["Success"] == True]
    time_in_segment = []

    for i in range(16):
        seg_df = filtered_df.loc[filtered_df["Segment"] == i]
        time = 0
        for j in range(seg_df.shape[0]):
            time = time + seg_df.iloc[j, 1] + seg_df.iloc[j, 2]
        time_in_segment.append(time)
    
    return time_in_segment


def estimate_avg_num_views_executed(data: List):
    for i in range(len(data)):
        data[i]['num_of_views_mean'] = statistics.mean(data[i]['num_of_views'])
        data[i]['num_of_views_std'] = statistics.stdev(data[i]['num_of_views'])

def estimate_avg_plan_exec(data: List):
    for i in range(len(data)):
        data[i]['num_of_plan_exec_mean'] = statistics.mean(data[i]['num_of_plan_exec'])
        data[i]['num_of_plan_exec_std'] = statistics.stdev(data[i]['num_of_plan_exec'])

def estimate_avg_plan_exec_success(data: List):
    for i in range(len(data)):
        data[i]['num_of_plan_exec_success_mean'] = statistics.mean(data[i]['num_of_plan_exec_success'])
        data[i]['num_of_plan_exec_success_std'] = statistics.stdev(data[i]['num_of_plan_exec_success'])

def estimate_avg_plan_time(data: List):
    for i in range(len(data)):
        data[i]['total_plan_time_mean'] = statistics.mean(data[i]['total_plan_time'])
        data[i]['total_plan_time_std'] = statistics.stdev(data[i]['total_plan_time'])

def estimate_avg_plan_time_success(data: List):
    for i in range(len(data)):
        data[i]['total_plan_time_success_mean'] = statistics.mean(data[i]['total_plan_time_success'])
        data[i]['total_plan_time_success_std'] = statistics.stdev(data[i]['total_plan_time_success'])

def estimate_avg_exec_time(data: List):
    for i in range(len(data)):
        data[i]['total_exec_time_mean'] = statistics.mean(data[i]['total_exec_time'])
        data[i]['total_exec_time_std'] = statistics.stdev(data[i]['total_exec_time'])

def estimate_avg_exec_time_success(data: List):
    for i in range(len(data)):
        data[i]['total_exec_time_success_mean'] = statistics.mean(data[i]['total_exec_time_success'])
        data[i]['total_exec_time_success_std'] = statistics.stdev(data[i]['total_exec_time_success'])

def estimate_avg_seg_time(data: List):
    for i in range(len(data)):
        data[i]['total_segment_time_mean'] = statistics.mean(data[i]['total_segment_time'])
        data[i]['total_segment_time_std'] = statistics.stdev(data[i]['total_segment_time'])

def estimate_high_seg_time(data: List):
    for i in range(len(data)):
        data[i]['highest_segment_time'] = max(data[i]['total_segment_time'])
        # print(data[i]['total_segment_time'].index(data[i]['highest_segment_time']))
        # raise("error")

def estimate_avg_seg_time_success(data: List):
    for i in range(len(data)):
        data[i]['total_segment_time_success_mean'] = statistics.mean(data[i]['total_segment_time_success'])
        data[i]['total_segment_time_success_std'] = statistics.stdev(data[i]['total_segment_time_success'])

def estimate_high_seg_time_success(data: List):
    for i in range(len(data)):
        data[i]['highest_segment_time_success'] = max(data[i]['total_segment_time_success'])


# method1_folders = ["/home/thesis/IROS/VMP/Scenario1/exp1/",
#                    "/home/thesis/IROS/VMP/Scenario1/exp2/",
#                    "/home/thesis/IROS/VMP/Scenario1/exp3/",
#                    "/home/thesis/IROS/VMP/Scenario1/exp4/",
#                    "/home/thesis/IROS/VMP/Scenario1/exp5/",
#                    "/home/thesis/IROS/VMP/Scenario1/exp6/",
#                    "/home/thesis/IROS/VMP/Scenario1/exp7/",
#                    "/home/thesis/IROS/VMP/Scenario1/exp8/",
#                    "/home/thesis/IROS/VMP/Scenario1/exp9/",
#                    "/home/thesis/IROS/VMP/Scenario1/exp10/"]
# method2_folders = ["/home/thesis/IROS/NBVSC/Scenario1/exp1/",
#                    "/home/thesis/IROS/NBVSC/Scenario1/exp2/",
#                    "/home/thesis/IROS/NBVSC/Scenario1/exp3/",
#                    "/home/thesis/IROS/NBVSC/Scenario1/exp4/",
#                    "/home/thesis/IROS/NBVSC/Scenario1/exp5/",
#                    "/home/thesis/IROS/NBVSC/Scenario1/exp6/",
#                    "/home/thesis/IROS/NBVSC/Scenario1/exp7/",
#                    "/home/thesis/IROS/NBVSC/Scenario1/exp8/",
#                    "/home/thesis/IROS/NBVSC/Scenario1/exp9/",
#                    "/home/thesis/IROS/NBVSC/Scenario1/exp10/"]
# method3_folders = ["/home/thesis/IROS/GOVMP/Scenario1/exp1/",
#                    "/home/thesis/IROS/GOVMP/Scenario1/exp2/",
#                    "/home/thesis/IROS/GOVMP/Scenario1/exp3/",
#                    "/home/thesis/IROS/GOVMP/Scenario1/exp4/",
#                    "/home/thesis/IROS/GOVMP/Scenario1/exp5/",
#                    "/home/thesis/IROS/GOVMP/Scenario1/exp6/",
#                    "/home/thesis/IROS/GOVMP/Scenario1/exp7/",
#                    "/home/thesis/IROS/GOVMP/Scenario1/exp8/",
#                    "/home/thesis/IROS/GOVMP/Scenario1/exp9/",
#                    "/home/thesis/IROS/GOVMP/Scenario1/exp10/"]
method1_folders = ["/home/thesis/IROS/VMP/Scenario2/exp1/",
                   "/home/thesis/IROS/VMP/Scenario2/exp2/",
                   "/home/thesis/IROS/VMP/Scenario2/exp3/",
                   "/home/thesis/IROS/VMP/Scenario2/exp4/",
                   "/home/thesis/IROS/VMP/Scenario2/exp5/",
                   "/home/thesis/IROS/VMP/Scenario2/exp6/",
                   "/home/thesis/IROS/VMP/Scenario2/exp7/",
                   "/home/thesis/IROS/VMP/Scenario2/exp8/",
                   "/home/thesis/IROS/VMP/Scenario2/exp9/",
                   "/home/thesis/IROS/VMP/Scenario2/exp10/"]
method2_folders = ["/home/thesis/IROS/NBVSC/Scenario2/exp1/",
                   "/home/thesis/IROS/NBVSC/Scenario2/exp2/",
                   "/home/thesis/IROS/NBVSC/Scenario2/exp3/",
                   "/home/thesis/IROS/NBVSC/Scenario2/exp4/",
                   "/home/thesis/IROS/NBVSC/Scenario2/exp5/",
                   "/home/thesis/IROS/NBVSC/Scenario2/exp6/",
                   "/home/thesis/IROS/NBVSC/Scenario2/exp7/",
                   "/home/thesis/IROS/NBVSC/Scenario2/exp8/",
                   "/home/thesis/IROS/NBVSC/Scenario2/exp9/",
                   "/home/thesis/IROS/NBVSC/Scenario2/exp10/"]
method3_folders = ["/home/thesis/IROS/GOVMP/Scenario2/exp1/",
                   "/home/thesis/IROS/GOVMP/Scenario2/exp2/",
                   "/home/thesis/IROS/GOVMP/Scenario2/exp3/",
                   "/home/thesis/IROS/GOVMP/Scenario2/exp4/",
                   "/home/thesis/IROS/GOVMP/Scenario2/exp5/",
                   "/home/thesis/IROS/GOVMP/Scenario2/exp6/",
                   "/home/thesis/IROS/GOVMP/Scenario2/exp7/",
                   "/home/thesis/IROS/GOVMP/Scenario2/exp8/",
                   "/home/thesis/IROS/GOVMP/Scenario2/exp9/",
                   "/home/thesis/IROS/GOVMP/Scenario2/exp10/"]


roi_coverage_file1 = [method1_folder + 'roi_coverage0.csv' for method1_folder in method1_folders]
planning_duration_file1 = [method1_folder + 'planning_duration0.csv' for method1_folder in method1_folders]
roi_coverage_file2 = [method2_folder + 'roi_coverage0.csv' for method2_folder in method2_folders]
planning_duration_file2 = [method2_folder + 'planning_duration0.csv' for method2_folder in method2_folders]
roi_coverage_file3 = [method3_folder + 'roi_coverage0.csv' for method3_folder in method3_folders]
planning_duration_file3 = [method3_folder + 'planning_duration0.csv' for method3_folder in method3_folders]

data_method1 = {'num_of_views': [], 'num_of_plan_exec': [], 'num_of_plan_exec_success': [],
                'total_plan_time': [], 'total_plan_time_success': [], 'total_exec_time': [],
                'total_exec_time_success': [], 'total_segment_time': [], 'highest_segment_time': [],
                'total_segment_time_success': []}
data_method2 = {'num_of_views': [], 'num_of_plan_exec': [], 'num_of_plan_exec_success': [],
                'total_plan_time': [], 'total_plan_time_success': [], 'total_exec_time': [],
                'total_exec_time_success': [], 'total_segment_time': [], 'highest_segment_time': [],
                'total_segment_time_success': []}
data_method3 = {'num_of_views': [], 'num_of_plan_exec': [], 'num_of_plan_exec_success': [],
                'total_plan_time': [], 'total_plan_time_success': [], 'total_exec_time': [],
                'total_exec_time_success': [], 'total_segment_time': [], 'highest_segment_time': [],
                'total_segment_time_success': []}

column_names = ["Segment", "Plan Duration(s)", "Execution Duration(s)", "Success"]

for i in range(len(method1_folders)):
    # 1. Number of viewposes executed - by counting the number of rows in all files
    df = pd.read_csv(roi_coverage_file1[i], skiprows=1, header=None)
    data_method1['num_of_views'].append(get_num_views_executed(df))

    df = pd.read_csv(planning_duration_file1[i], names=column_names, skiprows=1)
    # 2. Total number of planning-execution cycles
    data_method1['num_of_plan_exec'].append(get_num_plan_exec(df))

    # 3. Total number of successful planning-execution cycles
    data_method1['num_of_plan_exec_success'].append(get_num_plan_exec_success(df))

    # 4. Total planning time
    data_method1['total_plan_time'].append(get_plan_time(df))

    # 5. Total successful planning time
    data_method1['total_plan_time_success'].append(get_plan_time_sucess(df))

    # 6. Total execution time
    data_method1['total_exec_time'].append(get_exec_time(df))

    # 7. Total successful execution time
    data_method1['total_exec_time_success'].append(get_exec_time_success(df))

    # 8. Average time in a segment
    # 9. Highest time in a segment in any experiment
    data_method1['total_segment_time'].extend(get_segment_time(df))

    # 10. Average successful time in a segment
    # 11. Highest successful time in a segment in any experiment
    data_method1['total_segment_time_success'].extend(get_segment_time_success(df))


for i in range(len(method2_folders)):
    df = pd.read_csv(roi_coverage_file2[i], skiprows=1, header=None)
    # 1. Number of viewposes executed - by counting the number of rows in all files
    data_method2['num_of_views'].append(get_num_views_executed(df))

    df = pd.read_csv(planning_duration_file2[i], names=column_names, skiprows=1)
    # 2. Total number of planning-execution cycles
    data_method2['num_of_plan_exec'].append(get_num_plan_exec(df))

    # 3. Total number of successful planning-execution cycles
    data_method2['num_of_plan_exec_success'].append(get_num_plan_exec_success(df))

    # 4. Total planning time
    data_method2['total_plan_time'].append(get_plan_time(df))

    # 5. Total successful planning time
    data_method2['total_plan_time_success'].append(get_plan_time_sucess(df))

    # 6. Total execution time
    data_method2['total_exec_time'].append(get_exec_time(df))

    # 7. Total successful execution time
    data_method2['total_exec_time_success'].append(get_exec_time_success(df))

    # 8. Average time in a segment
    # 9. Highest time in a segment in any experiment
    data_method2['total_segment_time'].extend(get_segment_time(df))

    # 10. Average successful time in a segment
    # 11. Highest successful time in a segment in any experiment
    data_method2['total_segment_time_success'].extend(get_segment_time_success(df))


for i in range(len(method3_folders)):
    df = pd.read_csv(roi_coverage_file3[i], skiprows=1, header=None)
    # 1. Number of viewposes executed - by counting the number of rows in all files
    data_method3['num_of_views'].append(get_num_views_executed(df))

    df = pd.read_csv(planning_duration_file3[i], names=column_names, skiprows=1)
    # 2. Total number of planning-execution cycles
    data_method3['num_of_plan_exec'].append(get_num_plan_exec(df))

    # 3. Total number of successful planning-execution cycles
    data_method3['num_of_plan_exec_success'].append(get_num_plan_exec_success(df))

    # 4. Total planning time
    data_method3['total_plan_time'].append(get_plan_time(df))

    # 5. Total successful planning time
    data_method3['total_plan_time_success'].append(get_plan_time_sucess(df))

    # 6. Total execution time
    data_method3['total_exec_time'].append(get_exec_time(df))

    # 7. Total successful execution time
    data_method3['total_exec_time_success'].append(get_exec_time_success(df))

    # 8. Average time in a segment
    # 9. Highest time in a segment in any experiment
    data_method3['total_segment_time'].extend(get_segment_time(df))

    # 10. Average successful time in a segment
    # 11. Highest successful time in a segment in any experiment
    data_method3['total_segment_time_success'].extend(get_segment_time_success(df))


# 1. Number of viewposes executed - by counting the number of rows in all file
estimate_avg_num_views_executed([data_method1, data_method2, data_method3])

# 2. Total number of planning-execution cycles
estimate_avg_plan_exec([data_method1, data_method2, data_method3])

# 3. Total number of successfull planning-execution cycles
estimate_avg_plan_exec_success([data_method1, data_method2, data_method3])

# 4. Total planning time
estimate_avg_plan_time([data_method1, data_method2, data_method3])

# 5. Total successful planning time
estimate_avg_plan_time_success([data_method1, data_method2, data_method3])

# 6. Total execution time
estimate_avg_exec_time([data_method1, data_method2, data_method3])

# 7. Total successful execution time
estimate_avg_exec_time_success([data_method1, data_method2, data_method3])

# 8. Average time in a segment
estimate_avg_seg_time([data_method1, data_method2, data_method3])

# 9. Highest time in a segment in any experiment
estimate_high_seg_time([data_method1, data_method2, data_method3])

# 10. Average successful time in a segment
estimate_avg_seg_time_success([data_method1, data_method2, data_method3])

# 11. Highest successful time in a segment in any experiment
estimate_high_seg_time_success([data_method1, data_method2, data_method3])



plotnames = ["VMP -", "NBVSC -", "GOVMP -"]
data_methods = [data_method1, data_method2, data_method3]

print("************************Number of views executed in total************************")
for i in range(len(plotnames)):
    print(plotnames[i] + " Mean: " + str(data_methods[i]['num_of_views_mean']) + " Std: " + str(data_methods[i]['num_of_views_std']))

print("************************Number of planning-execution cycles in total************************")
for i in range(len(plotnames)):
    print(plotnames[i] + " Mean: " + str(data_methods[i]['num_of_plan_exec_mean']) + " Std: " + str(data_methods[i]['num_of_plan_exec_std']))

print("************************Number of successful planning-execution cycles in total************************")
for i in range(len(plotnames)):
    print(plotnames[i] + " Mean: " + str(data_methods[i]['num_of_plan_exec_success_mean']) + " Std: " + str(data_methods[i]['num_of_plan_exec_success_std']))

print("************************Total planning time************************")
for i in range(len(plotnames)):
    print(plotnames[i] + " Mean: " + str(data_methods[i]['total_plan_time_mean']) + " Std: " + str(data_methods[i]['total_plan_time_std']))

print("************************Total successful planning time************************")
for i in range(len(plotnames)):
    print(plotnames[i] + " Mean: " + str(data_methods[i]['total_plan_time_success_mean']) + " Std: " + str(data_methods[i]['total_plan_time_success_std']))

print("************************Total execution time************************")
for i in range(len(plotnames)):
    print(plotnames[i] + " Mean: " + str(data_methods[i]['total_exec_time_mean']) + " Std: " + str(data_methods[i]['total_exec_time_std']))

print("************************Total successful execution time************************")
for i in range(len(plotnames)):
    print(plotnames[i] + " Mean: " + str(data_methods[i]['total_exec_time_success_mean']) + " Std: " + str(data_methods[i]['total_exec_time_success_std']))

print("************************Average time per segment************************")
for i in range(len(plotnames)):
    print(plotnames[i] + " Mean: " + str(data_methods[i]['total_segment_time_mean']) + " Std: " + str(data_methods[i]['total_segment_time_std']))

print("************************Highest time per segment************************")
for i in range(len(plotnames)):
    print(plotnames[i] + " Highest: " + str(data_methods[i]['highest_segment_time']))

print("************************Average successful time per segment************************")
for i in range(len(plotnames)):
    print(plotnames[i] + " Mean: " + str(data_methods[i]['total_segment_time_success_mean']) + " Std: " + str(data_methods[i]['total_segment_time_success_std']))

print("************************Highest successful time per segment************************")
for i in range(len(plotnames)):
    print(plotnames[i] + " Highest: " + str(data_methods[i]['highest_segment_time_success']))