import bsl_scripter
import serial
from binascii import hexlify

DATA_ACK = 0x90
DATA_NAK = 0xA0
LOAD_PC  = 0x1A

serListener = serial.Serial('COM6', timeout = 0.1)
print('Listening at port: ', serListener.name)

while(1):
    frame = bsl_scripter.get_frame(serListener)
    print(hexlify(frame.to_bytes()))
    if frame.verify_cksum():
        print('Sending DATA_ACK')
        serListener.write(bytes([DATA_ACK]))
    else:
        print('Sending DATA_NAK')
        serListener.write(bytes([DATA_NAK]))
    if frame.cmd == LOAD_PC:
        break

serListener.close()
