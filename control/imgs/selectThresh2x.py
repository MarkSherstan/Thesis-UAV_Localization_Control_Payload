from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

dataFiles = ['Data0', 'Data1', 'Data2', 'Data3', 'Data4']
dataFrameList = []

for dat in dataFiles:
    # Read csv into data frame
    df = pd.read_csv('2x/'+dat+'.csv', header=0, names=['adaptiveThreshWinSizeMin', 'adaptiveThreshWinSizeMax',
                                                'adaptiveThreshWinSizeStep', 'foundA', 'foundB', 'Total Found', 
                                                'Thresh1', 'Thresh2', 'Cam A Imgs', 'Cam B Imgs'])

    # Create some additional metrics
    df['Percent-Tot'] = (df['foundA'] + df['foundB']) / (df['Cam A Imgs'] + df['Cam B Imgs']) 
    df['THRESH'] = df.Thresh1.astype(str) + '-' + df.Thresh2.astype(str)

    # Adjust everything to 100%
    tempMax = df['Percent-Tot'].max()
    df['Scaled-Percent'] = df['Percent-Tot'] / tempMax
    
    # Store the data
    dataFrameList.append(df)

# Combine all data frames into one
df = pd.concat(dataFrameList)


# Plot data
# fig = plt.figure(figsize=(8, 4), dpi=100)
# ax = plt.gca()
# df.plot(kind='line', x='Thresh1', y='Scaled-Percent', style='.', ax=ax)
# plt.show()




# Average everything based off thresh 1 value
tempData = df.groupby('Thresh1').mean()

# Save data into numpy array
idx = np.array(tempData.index)
x = np.array(tempData['Thresh2'])
y = np.array(tempData['Scaled-Percent'])

# Create plot
fig = plt.figure(figsize=(10, 5), dpi=100)
ax = plt.gca()

# Plot the data
for ii in range(len(idx)):
    if not(ii % 3):
        ax.scatter(x[ii], y[ii], marker='*', label=idx[ii])
    else:
        ax.scatter(x[ii], y[ii], marker='.', label=idx[ii])
        
# Format
ax.set_title('Threshold Values vs Accuracy', fontsize=14, fontweight='bold')
ax.set_xlabel('Threshold 2', fontweight='bold')
ax.set_ylabel('Accuracy [%]',  fontweight='bold')
ax.legend(ncol=8, loc='lower right')

plt.show()



# temp = df.groupby('THRESH')['Percent-Tot'].mean()
# print(temp)
# exit()

# # Find the top 20%
# top = df.nlargest(int(len(df)*(20/100)), 'Percent-Tot')

# dataList.append(top['THRESH'].value_counts()[:10].index.tolist())


# merged = list(itertools.chain(*dataList))
# A = list(set(merged))

# print(A)

# # print(df['THRESH'].nunique())








# z = np.array(df['Percent-Tot'])
# x = np.array(df['Thresh1'])
# y = np.array(df['Thresh2'])

# fig = plt.figure()
# ax = Axes3D(fig)

# ax.scatter(x, y, z)

# ax.set_xlabel('Thresh 1')
# ax.set_ylabel('Thresh 2')
# ax.set_zlabel('Percent Accuracy')

# plt.show()
