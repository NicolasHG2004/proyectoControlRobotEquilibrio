import serial
import pandas as pd

PUERTO = 'COM12'
BAUDRATE = 115200
N_MUESTRAS = 500

ser = serial.Serial(PUERTO,BAUDRATE, timeout=1)
print(f"Leyendo {N_MUESTRAS} muestras")

datos = []

for _ in range(N_MUESTRAS):
    linea = ser.readline().decode('utf-8', errors='ignore').strip()
    print(linea)

    try:
        partes = linea.split()
        fila ={}
        for parte in partes:
            clave, valor = parte.split(':')
            fila[clave] = int(valor)
        if fila:
            datos.append(fila)
    except:
        pass
    ser.close()
    df = pd.DataFrame(datos)
    df.to_excel('datosProyectoControl.xlsx', index=False)
    print("Guardados")