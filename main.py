from trossen import Trossen

DEVICE_NAME = '/dev/cu.usbserial-FT6Z5MSQ'
BAUD_RATE = 1000000

def main():
    trossen = Trossen(port=DEVICE_NAME, baudrate=BAUD_RATE)

    while 1:
        user_input = input("Enter a position: ")
        if user_input == "q":
            break
        try:
            trossen.setEndEffectorPos(int(user_input))
            continue
        except ValueError:
            pass

    trossen.close()

if __name__ == '__main__':
    main()
