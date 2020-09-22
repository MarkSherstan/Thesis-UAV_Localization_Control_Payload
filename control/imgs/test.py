count = 0

for adaptiveThreshWinSizeMin in range(3, 30):
    for adaptiveThreshWinSizeMax in range(3, 30):
        for adaptiveThreshWinSizeStep in range(3, 30):
            # Lazy assert
            min3 = (adaptiveThreshWinSizeMin >= 3) and (adaptiveThreshWinSizeMax >= 3)
            maxMin = adaptiveThreshWinSizeMax >= adaptiveThreshWinSizeMin
            not0 = adaptiveThreshWinSizeStep > 0
            
            # Perform calc
            if (min3 and maxMin and not0):
                nScales =  int((adaptiveThreshWinSizeMax - adaptiveThreshWinSizeMin) / adaptiveThreshWinSizeStep + 1)
                
                if (nScales == 2):
                    count += 1
            
                    for ii in range(nScales):
                        currScale = adaptiveThreshWinSizeMin + ii * adaptiveThreshWinSizeStep


print(count)

# minVal = adaptiveThreshWinSizeMin = 5   
# maxVal = adaptiveThreshWinSizeMax = 21 
# step   = adaptiveThreshWinSizeStep = 4

# nScales =  int((maxVal - minVal) / step + 1)

# for ii in range(nScales):
#     currScale = minVal + ii * step
#     print(currScale)
