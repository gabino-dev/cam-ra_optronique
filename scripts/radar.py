#!/home/gabin/venvs/pyside-env/bin/python
import time
import serial

port = serial.Serial('/dev/pts/3', 4800)

while True:
    sentence = "$TTM,0.054,N,090.0,T,010.0,090.0,Target1,A*24\r\n"
    port.write(sentence.encode())
    time.sleep(1)
