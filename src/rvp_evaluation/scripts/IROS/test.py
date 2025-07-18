import pandas as pd
import statistics

column_names = ["Segment", "Plan Duration(s)", "Execution Duration(s)", "Success"]
df = pd.read_csv("/home/thesis/IROS/NBVSC/Scenario2/exp1/planning_duration0.csv", names=column_names, skiprows=1)

print("number of total planning-execution cycles: " + str(df.shape[0]))

filtered_df = df.loc[df["Success"] == True]

print("number of successfull planning-execution cycles: " + str(filtered_df.shape[0]))


print("total time taken for planning: " + str(df['Plan Duration(s)'].sum()))

print("total time taken for successfull planning: " + str(filtered_df['Plan Duration(s)'].sum()))

print("total time taken for execution: " + str(df['Execution Duration(s)'].sum()))

print("total time taken for successfull execution: " + str(filtered_df['Execution Duration(s)'].sum()))

time_in_segment = []

for i in range(16):
    seg_df = df.loc[df["Segment"] == i]
    time = 0
    for j in range(seg_df.shape[0]):
        time = time + seg_df.iloc[j, 1] + seg_df.iloc[j, 2]
    time_in_segment.append(time)

print(time_in_segment)
print("average time in a segment: " + str(statistics.mean(time_in_segment)))
print("highest time in a segment: " + str(max(time_in_segment)))


time_in_segment = []

for i in range(16):
    seg_df = filtered_df.loc[filtered_df["Segment"] == i]
    time = 0
    for j in range(seg_df.shape[0]):
        time = time + seg_df.iloc[j, 1] + seg_df.iloc[j, 2]
    time_in_segment.append(time)

print(time_in_segment)
print("average time in a segment with successfull planning-execution: " + str(statistics.mean(time_in_segment)))
print("highest time in a segment with successfull planning-execution: " + str(max(time_in_segment)))