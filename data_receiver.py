from serial import Serial
import argparse, time, struct
parser = argparse.ArgumentParser(description="Retrieves data from the camera controller")
parser.add_argument('port', type=str, help='The port to open')

def get_packet(device):
    while device.read(1)[0] != 0xff:
        pass
    raw_data = device.read(14)
    if raw_data[-1] != 0xaa:
        return None
    else:
        packet = struct.unpack('IfI?', raw_data[0:-1])
        return packet

if __name__ == "__main__":
    args = parser.parse_args()
    device = Serial(port=args.port, baudrate=115200)
    time.sleep(5)
    device.write(bytearray('start\n', 'ASCII'))
    raw_start_time = device.read(4)
    start_time = struct.unpack('I', raw_start_time)
    print("start time is {}".format(start_time))
    try :
        while True:
            data_tuple = get_packet(device)
            print(data_tuple)
    except BaseException as e:
        device.close()
        raise(e)
