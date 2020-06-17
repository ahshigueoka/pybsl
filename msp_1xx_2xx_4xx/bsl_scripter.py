from binascii import hexlify
import serial

# Data frame header
TX_REQUEST      = 0x10
BSL_REQUEST     = 0xB0
BSL_HEADER      = 0x80

# Command codes
RX_DATA_BLOCK   = 0x12
RX_PASSWORD     = 0x10
ERASE_SEGMENT   = 0x16
ERASE_MASS      = 0x18
ERASE_CHECK     = 0x1C
CHANGE_BDRATE   = 0x20
SET_MEMOFFSET   = 0x21
LOAD_PC         = 0x1A
TX_DATA_BLOCK   = 0x14
TX_BSL_VER      = 0x1E

# Responses from the target
DATA_ACK        = 0x90
DATA_NAK        = 0xA0

def erase_app():
    dtf = BSLframe(BSL_HEADER, ERASE_MASS, 4, 0x0000, 0x06, 0xA5, [], 0, 0)
    dtf.update_cksum()
    return dtf

def erase_segment(addr):
    dtf = BSLframe(BSL_HEADER, ERASE_SEGMENT, 4, addr, 0x02, 0xA5, [], 0, 0)
    dtf.update_cksum()
    return dtf

def load_pc(addr):
    # Address must be supplied as an integer, with at most 16 bits
    dtf = BSLframe(BSL_HEADER, LOAD_PC, 4, addr, 0, 0, [], 0, 0)
    dtf.update_cksum()
    return dtf

def tx_data_block(addr, numbt):
    dtf = BSLframe(BSL_HEADER, TX_DATA_BLOCK, 4, addr, numbt, 0, [], 0, 0)
    dtf.update_cksum()
    return dtf

def bsl_request():
    dtf = BSLframe(BSL_REQUEST, 0, 4, 0x0000, 0x00, 0x00, [], 0x00, 0x00)
    dtf.update_cksum()
    return dtf

def parse_titxt(imageFile):
    """ Accepts the image file of some compiled code in TI-TXT and separates the address and the hexadecimal data.

    :param imageFile: the name of the image file to be processed.
    :type imageFile: Python string.

    :return: list of tuples, where the first element of the tuple is the starting address and the second element is the hexadecimal data to be saved.
    :rtype: list of two-element tuples. For each tuple, the first element is a Python string and the second is a list of strings
    """

    # Initialize variables
    startAddress = []
    ptree_byte = []
    hexData = []

    # Open the image file and parse it
    with open(imageFile, 'r') as fh:
        # Read next line. Should never have more than 51 characters
        currentLine = fh.readline(100).strip(' \t\r\n')
        # Keep reading while there is content to process
        while currentLine:
            # Check the type of the instruction:
            if currentLine[0] == '@':
                # The command read is an address
                # Save the old record to start a new one,
                # if the old record exists
                if startAddress:
                    ptree_byte.append((startAddress, hexData))
                # Store the address at another variable
                startAddress = bytearray.fromhex(currentLine[1:])
                # Start a new sequence of hex data
                hexData = bytearray();
            elif currentLine[0] == 'q':
                # The end of the image file has been reached
                # Save the old record
                ptree_byte.append((startAddress, hexData))
                # Stop iterating over the file.
                break;
            else:
                # There is more data to load
                # Append the hex data by extending the list
                hexData.extend(bytearray.fromhex(currentLine))
            # Read next line
            currentLine = fh.readline(100).strip(' \t\r\n')
        else:
            # Exhausted the file and could not find a 'q'.
            # Issue a warning.
            print("Warning: file ended without an End-Of-File charactaer 'q'. Assuming the image file has finished and can be saved.")
            if startAddress:
                    ptree_byte.append((startAddress, hexData))

        return ptree_byte

def convert_rxdata(ptree_byte, maxchunk = 16, showDump = False):
    commandList = [];

    for (stAdd, hexData) in ptree_byte:
        bslCommands = convert_rxchunk(stAdd, hexData, maxchunk)
        commandList.extend(bslCommands)

    if showDump:
        print('----------------------------------------')
        print('Dump of image file:')
        for command in commandList:
            print(hexlify(command.to_bytes()))
        print('----------------------------------------\n')

    return commandList

def convert_rxchunk(startAddress, hexData, maxchunk = 16):
    addressSize = len(startAddress)
    # NOTE: the address is given in big-endian convention in the TI-TXT file
    chunkAddress = int.from_bytes(startAddress, byteorder = 'big', signed = False)
    dataSize = len(hexData)
    dataIt = 0;
    rxFrames = [];
    # Create the RX data frames and append to the list rxFrames
    while dataIt < dataSize:
        chunkSize = min(dataSize - dataIt, maxchunk)
        chunkEnd = dataIt + chunkSize
        chunkData = list(hexData[dataIt:chunkEnd])
        rxCommand = rx_data_block(chunkAddress, chunkData)
        rxFrames.append(rxCommand)
        dataIt = dataIt + maxchunk
        chunkAddress = chunkAddress + chunkSize

    return rxFrames

def dump_print(barray, startAddress, bytesPerLine = 16):
    numbytes = len(barray)

    print('-----------------------------')
    print('Image size: ', numbytes, ' bytes')
    for it in range(numbytes):
        if (it % bytesPerLine) == 0:
            print('\n', hex(startAddress + it), end = ': ')
            lineprint = bytearray(barray[it:(it + bytesPerLine)])
            print(hexlify(lineprint), end = '')

    print('\n-----End of image dump-----', end = '\n\n')

def rx_data_block(addr, data):
    # Length of payload: number of bytes in data plus
    # two bytes plus two bytes for extra parameters
    dtframe = BSLframe(BSL_HEADER, RX_DATA_BLOCK, len(data) + 4, addr, len(data), 0, data, 0, 0)
    dtframe.update_cksum()

    return dtframe

def checksum_xor(data):
    data_size = len(data)
    if len(data) == 1:
        return [data, 0]

    ckl = data[0]
    ckh = data[1]
    for j in range(2, data_size, 2):
        ckl = ckl^data[j]

    for j in range(3, data_size, 2):
        ckh = ckh^data[j]

    return [~ckl & 0xFF, ~ckh & 0xFF]

def get_frame(so):
    hdr = int.from_bytes(so.read(), byteorder = 'little')
    cmd = int.from_bytes(so.read(), byteorder = 'little')
    l1 = int.from_bytes(so.read(), byteorder = 'little')
    l2 = int.from_bytes(so.read(), byteorder = 'little')
    pl = so.read(l1)
    address = int.from_bytes(pl[0:2], byteorder = 'little')
    arg1 = int.from_bytes(pl[2], byteorder = 'little')
    arg2 = int.from_bytes(pl[3], byteorder = 'little')
    dat  = list(pl[3:end])
    ckl = int.from_bytes(so.read(), byteorder = 'little')
    ckh = int.from_bytes(so.read(), byteorder = 'little')
    return BSLframe(hdr, cmd, l1, address, arg1, arg2, dat, ckl, ckh)

class BSLframe:
    def __init__(self, hdr, cmd, plsize, address, arg1, arg2, dat, ckl, ckh):
        self.hdr     = hdr
        self.cmd     = cmd
        self.plsize  = plsize
        self.address = address
        self.arg1    = arg1
        self.arg2    = arg2
        self.dat     = dat
        self.ckl     = ckl
        self.ckh     = ckh

    def to_bytes(self):
        self.update_cksum()
        command = bytearray(self.concat_params() + [self.ckl, self.ckh])
        return command

    def concat_params(self):
        arr = [self.hdr, self.cmd, self.plsize, self.plsize] \
              + [self.address & 0xFF, self.address >> 8] \
              + [self.arg1, self.arg2] \
              + self.dat
        return arr

    def verify_cksum(self):
        cksum = checksum_xor(self.concat_params())
        if(self.ckl == cksum[0] and self.ckh == cksum[1]):
            return True
        else:
            return False

    def update_cksum(self):
        cksum = checksum_xor(self.concat_params())
        self.ckl = cksum[0]
        self.ckh = cksum[1]

class Host:
    """Performs the communication between a computer and a target MSP430
    running a MSPBoot bootloader"""
    def __init__(self, comport):
        """


        Parameters
        ----------
        comport : string
            name of the comport the target MSP430 is connected to .

        Returns
        -------
        None.

        """
        self.ptree = []
        self.flextree = []
        self.so = serial.Serial(comport, 9600, timeout = 10)

    def __enter__(self):
        """Required to use the 'with' statement with this class"""
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Ensures that the serial object will be closed when
        an instance of this object is destroyed"""
        self.so.close()

    def load_image(self, imageFile, showDump = False):
        # Get the list of RX frames
        print('Reading TI-TXT image file')
        self.ptree = parse_titxt(imageFile)
        self.rxdata = convert_rxdata(self.ptree, 16, showDump)

    def write_image(self):
        print('Deleting image at the target')
        print(hexlify(erase_app().to_bytes()))
        print('Writing firmware')
        for command in (self.rxdata):
            #self.so.write(command.to_bytes())
            print(hexlify(command.to_bytes()))

    def invoke_bsl(self):
        print('Starting BSL')
        print(bsl_request().to_bytes())
        self.so.write(bsl_request().to_bytes())
        print('Waiting for acknowledge')
        NUM_ATTEMPTS = 10
        for j in range(0, NUM_ATTEMPTS):
            temp = self.so.read()
            ans = int.from_bytes(temp, 'little', signed = False)
            print('Received char:', end = ' ')
            print(hexlify(temp))
            if ans == DATA_ACK:
                print('Entered BSL mode')
                return True
            elif ans == DATA_NAK:
                print('Failed to enter BSL mode')
                return False
        print('No response')

    def jump2app(self, addr):
        # Send a jump to app
        print('Starting app')
        print(hexlify(load_pc(addr).to_bytes()))
