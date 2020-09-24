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



# Create plot
fig = plt.figure(figsize=(10, 5), dpi=100)
ax = plt.gca()

# Loop through data idx's
for ii in range(3,27):
    # Get data 
    temp = df.loc[df['Thresh1'] == ii]
    x = np.array(temp['Thresh2'])
    y = np.array(temp['Scaled-Percent'])

    # Plot the data
    if not(ii % 3):
        ax.scatter(x, y, marker='*', label=ii)
    elif not(ii % 2):
        ax.scatter(x, y, marker='o', label=ii)
    else:
        ax.scatter(x, y, marker='.', label=ii)
        
# Format
ax.set_title('Threshold Values vs Accuracy', fontsize=14, fontweight='bold')
ax.set_xlabel('Threshold 2', fontweight='bold')
ax.set_ylabel('Accuracy [%]',  fontweight='bold')
ax.legend(ncol=8, loc='lower right')

plt.show()



# # Average everything based off thresh 1 value
# tempData = df.groupby('Thresh1').mean()

# # Save data into numpy array
# idx = np.array(tempData.index)
# x = np.array(tempData['Thresh2'])
# y = np.array(tempData['Scaled-Percent'])

# # Create plot
# fig = plt.figure(figsize=(10, 5), dpi=100)
# ax = plt.gca()

# # Plot the data
# for ii in range(len(idx)):
#     if not(ii % 3):
#         ax.scatter(x[ii], y[ii], marker='*', label=idx[ii])
#     else:
#         ax.scatter(x[ii], y[ii], marker='.', label=idx[ii])
        
# # Format
# ax.set_title('Threshold Values vs Accuracy', fontsize=14, fontweight='bold')
# ax.set_xlabel('Threshold 2', fontweight='bold')
# ax.set_ylabel('Accuracy [%]',  fontweight='bold')
# ax.legend(ncol=8, loc='lower right')

# plt.show()




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
