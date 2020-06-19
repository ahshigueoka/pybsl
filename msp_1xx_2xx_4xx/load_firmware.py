# -*- coding: utf-8 -*-
import bsl_scripter
from binascii import hexlify

#=============================================================================
# Configuration section
# Change the variable according to your needs

# Address in the target MSP where the CRC16 CCITT of the firmware is stored
# must be specified as a 'bytes' type
#
# The five addresses below must be configured according to the values defined
# in the linker file of the MSP430
crc_address = bytearray(b'\x00\x44\x00')
app_start   = 0x04402
app_end     = 0x0F3FF
flx_start   = 0x10000
flx_end     = 0x149FF

#-----------------------------------------------------------------------------
# Change the string below according to the COM port used at the host (PC) side
comport = 'COM50'

#-----------------------------------------------------------------------------
# Change the file name to the TI-TXT file you want to write
imageFile = 'examples/app2_new.txt'

#-----------------------------------------------------------------------------
# If write_flag ==  True, then write the firmware
# If write_flag == False, then just load and calculate the checksum
write_flag  = True

#-----------------------------------------------------------------------------
# Max number of attempts to start MSPBoot
max_attempts_mspboot = 10;

#=============================================================================
# Write section
#
# From this point it should not be mandatory to modify this script.
print('Opening COM port')
# Open UART communication
pchost = bsl_scripter.Host(comport)
# Load firmware from image file
pchost.load_image('examples/app2_new.txt')

# Start sequence
if write_flag:
    print("Attempting to start BSL at the target device")
    if pchost.invoke_bsl():
        # Overwrite the firmware at the target
        pchost.mass_erase()
        pchost.write_image()
        # Start executing the target application
        pchost.load_pc(app_start)
        pchost.exit_bsl()
    else:
        print('Could not communicate with the target MSP430')
