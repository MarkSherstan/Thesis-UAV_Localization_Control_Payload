import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import itertools

# Data files 
dataFiles = ['Data0', 'Data1', 'Data2', 'Data3', 'Data4', 'Data5']
colors = ['r', 'g', 'b', 'c', 'm', 'k']

# Create a figure 
fig = plt.figure(figsize=(8, 4), dpi=100)
ax = plt.gca()

# Loop through results
for ii in range(len(dataFiles)):
    # Read csv into data frame
    df = pd.read_csv(dataFiles[ii]+'.csv', header=0, names=['adaptiveThreshWinSizeMin', 'adaptiveThreshWinSizeMax',
                                        'adaptiveThreshWinSizeStep', 'foundA', 'foundB', 'Total Found', 
                                        'Thresh', 'Cam A Imgs', 'Cam B Imgs'])

    # Create some additional metrics
    df['Percent-Tot'] = (df['foundA'] + df['foundB']) / (df['Cam A Imgs'] + df['Cam B Imgs']) 
    
    # Plot data
    df.plot(kind='line', x='Thresh', y='Percent-Tot', style='.', color=colors[ii], ax=ax)

# Format
ax.set_title('Threshold Vs Accuracy', fontsize=14, fontweight='bold')
ax.set_xlabel('Threshold Value', fontweight='bold')
ax.set_ylabel('Relative Accuracy [%]',  fontweight='bold')
ax.legend(dataFiles)

plt.show()
