import struct
from pySerialTransfer import pySerialTransfer as txfer


if __name__ == '__main__':
    try:
        link = txfer.SerialTransfer('/dev/ttyUSB0')
        link.open()
        
        while True:
            if link.available():
                data1 = struct.unpack('f', bytes(link.rxBuff[0:4]))[0]
                data2 = struct.unpack('f', bytes(link.rxBuff[4:8]))[0]
                data3 = struct.unpack('f', bytes(link.rxBuff[8:12]))[0]
                data4 = struct.unpack('f', bytes(link.rxBuff[12:16]))[0]
                
                print('data1: {}\ndata2: {}\ndata3: {}\ndata4: {}\n'.format(data1,
                                                                            data2,
                                                                            data3,
                                                                            data4))
            
            elif link.status < 0:
                    print('ERROR: {}'.format(link.status))
        
    except KeyboardInterrupt:
        link.close()
