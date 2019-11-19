import serial
import struct
import time

serialPort = '/dev/cu.wchusbserial1410'
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
            ser.write(b'\xFE')
            print('Open')
        elif (str == 'c'):
            ser.write(b'\xFF')
            print('Close')
        elif (str == 'cap'):
            ser.write(b'\x01')
            print('Cap request')

            # print(hex(struct.unpack('B', ser.read())[0]))
            while(True):
                var = ser.read()
                print( hex(struct.unpack('B', var)[0]))
                var = struct.unpack('B', var)[0]

                if var is 90:
                    ser.write(b'\x5A')
                    break

            print('YAY')

            while(True):
                print(hex(struct.unpack('B', ser.read())[0]))
                str = input('In: ')

                # Send to servo
                if (str == ' '):
                    ser.write(b'\x0A')



                # print('hello')
                # try:
                #     print(hex(struct.unpack('B', ser.read())[0]))
                # except:
                #     print('-')
                #     break
        else:
            print('Invalid input')


        # Reading
        try:
            print(hex(struct.unpack('B', ser.read())[0]))
        except:
            print('-')

    except:
        # Exit sequence
        ser.close()
        print('End...')
        break
