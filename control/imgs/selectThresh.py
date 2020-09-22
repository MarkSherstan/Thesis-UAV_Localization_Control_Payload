import pandas as pd


df = pd.read_csv('Data.csv', header=0, names=['adaptiveThreshWinSizeMin', 'adaptiveThreshWinSizeMax',
                                              'adaptiveThreshWinSizeStep', 'foundA', 'foundB', 'Total Found', 
                                              'Thresh1', 'Thresh2', 'Cam A Imgs', 'Cam B Imgs'])

top25 = df.nlargest(25,'Total Found')

print(top25)


# Plot threshold vs percentage correct?