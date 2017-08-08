from serial import Serial
from xbee import XBee

ser=Serial('COM11','9600')
ser.write(b'Hello world!\n')
ser.read(4)#reads 4 bytes


#for frames
xb =XBee(ser)
xb.send(command='Hello world!')
print xb.wait_read_frame()#waits on a command to come in




