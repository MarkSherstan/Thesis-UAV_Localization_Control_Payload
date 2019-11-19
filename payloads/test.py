import serial
import time

serialPort = '/dev/cu.wchusbserial1420'
serialBaud = 9600

# Connect to serial port
try:
    ser = serial.Serial(serialPort, serialBaud)
    print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
except:
    print('Failed to connect with ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')

# Small delay
time.sleep(1)

# Run till force quit
while True:
    try:
        # Get input
        str = input('Open [o] or Close [c]: ')

        # Send to servo
        if (str == 'o'):
            ok = ser.write(b'\xFE')
            print('Open', ok)
        elif (str == 'c'):
            ok = ser.write(b'\xFF')
            print('Close ', ok)
        else:
            print('Invalid input')
    except:
        # Exit sequence
        ser.close()
        print('End...')
        break
