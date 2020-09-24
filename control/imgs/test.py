
adaptiveThreshWinSizeMin = 49
adaptiveThreshWinSizeMax = 53
adaptiveThreshWinSizeStep = 3



nScales = int((adaptiveThreshWinSizeMax - adaptiveThreshWinSizeMin) / adaptiveThreshWinSizeStep + 1)

for ii in range(nScales):
    a = adaptiveThreshWinSizeMin + ii*adaptiveThreshWinSizeStep
    print(ii, nScales, a)
    
