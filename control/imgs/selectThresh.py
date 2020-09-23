import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv('Data.csv', header=0, names=['adaptiveThreshWinSizeMin', 'adaptiveThreshWinSizeMax',
                                              'adaptiveThreshWinSizeStep', 'foundA', 'foundB', 'Total Found', 
                                              'Thresh1', 'Thresh2', 'Cam A Imgs', 'Cam B Imgs'])

df['Percent-Tot'] = (df['foundA'] + df['foundB']) / (df['Cam A Imgs'] + df['Cam B Imgs']) 
df['Percent-A'] = df['foundA'] / df['Cam A Imgs'] 
df['Percent-B'] = df['foundB'] / df['Cam B Imgs']

top25 = df.nlargest(25, 'Percent-Tot')

print(top25)


# Time Plot
fig = plt.figure(figsize=(8, 4), dpi=100)
ax = plt.gca()

# Plot data
top25.plot(kind='line', x='Thresh1', y='Percent-Tot', style='.', color='k', ax=ax)

# Format
ax.set_title('Test', fontsize=14, fontweight='bold')
ax.set_xlabel('Time [s]', fontweight='bold')
ax.set_ylabel('dt [s]',  fontweight='bold')
ax.get_legend().remove()


plt.show()

# Plot threshold vs percentage correct?
# Sort by top camera 1, camera 2, total, and percentages???
# 3D plot???
# mode of the top 50 or something?