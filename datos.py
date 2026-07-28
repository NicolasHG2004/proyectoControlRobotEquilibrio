import serial

ser = serial.Serial('COM20',115200,timeout=1)

while True:
    linea = ser.readline().decode('utf-8',errors='ignore').strip()

    print(repr(linea))