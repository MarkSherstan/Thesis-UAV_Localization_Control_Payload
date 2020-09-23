import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Read csv into data frame
df = pd.read_csv('Data0.csv', header=0, names=['adaptiveThreshWinSizeMin', 'adaptiveThreshWinSizeMax',
                                              'adaptiveThreshWinSizeStep', 'foundA', 'foundB', 'Total Found', 
                                              'Thresh1', 'Thresh2', 'Cam A Imgs', 'Cam B Imgs'])

# Create some additional metrics
df['Percent-Tot'] = (df['foundA'] + df['foundB']) / (df['Cam A Imgs'] + df['Cam B Imgs']) 
df['Percent-A'] = df['foundA'] / df['Cam A Imgs'] 
df['Percent-B'] = df['foundB'] / df['Cam B Imgs']
df['THRESH'] = df.Thresh1.astype(str) + '-' + df.Thresh2.astype(str)

# Find the top 20%
top20percent = df.nlargest(int(len(df)*(20/100)), 'Percent-Tot')
print(top20percent)

# print(top50['THRESH'].value_counts()[:10].index.tolist())

print(df['THRESH'].nunique())


# # Time Plot
# fig = plt.figure(figsize=(8, 4), dpi=100)
# ax = plt.gca()

# # Plot data
# df.plot(kind='line', x='THRESH', y='Percent-Tot', style='.', color='k', ax=ax)

# # Format
# ax.set_title('Test', fontsize=14, fontweight='bold')
# ax.set_xlabel('Time [s]', fontweight='bold')
# ax.set_ylabel('dt [s]',  fontweight='bold')
# ax.get_legend().remove()


# plt.show()



# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# x = np.array(df['Thresh1'])
# y = np.array(df['Thresh2'])
# z = np.array(df['Percent-Tot'])

# ax.scatter(x, y, z)

# ax.set_xlabel('Thresh 1')
# ax.set_ylabel('Thresh 2')
# ax.set_zlabel('Percent Accuracy')

# plt.show()


# Plot threshold vs percentage correct?
# Sort by top camera 1, camera 2, total, and percentages???
# 3D plot???
# mode of the top 50 or something?