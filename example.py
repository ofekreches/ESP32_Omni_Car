import serial
import time
import struct

# Constants for communication protocol
HEADER = 200
TAIL = 199
SIZE_OF_RX_DATA = 17
VELOCITY_MODE = 1  

# Serial port configuration
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
TIMEOUT = 1

serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)

def calculate_receive_checksum(data):
    """Calculate checksum for received data."""
    return sum(data[2:50]) % 256  

def calculate_transmit_checksum(data):
    """Calculate checksum for data to be transmitted."""
    return sum(data[3:11]) % 256

def send_velocity_command(velocity):
    """Send velocity command to the omni-wheel car."""
    data = bytearray(SIZE_OF_RX_DATA)
    data[0:2] = HEADER, HEADER
    data[2] = VELOCITY_MODE

    # Packing velocity into 4 bytes
    data[3:7] = struct.pack('f', velocity)  # velocity x
    # data[7:11] = velocity_bytes # velocity y
    # data[11:15] = velocity_bytes #velocity angular z
    checksum = calculate_transmit_checksum(data)
    data[SIZE_OF_RX_DATA - 2] = checksum
    data[SIZE_OF_RX_DATA - 1] = TAIL
    serial_conn.write(data)

def read_serial_data():
    """Read data from serial and process it."""
    if serial_conn.in_waiting > 0:
        received_data = serial_conn.read(serial_conn.in_waiting)
        if len(received_data) >= 52 and received_data[0:2] == (HEADER, HEADER):
            if calculate_receive_checksum(received_data) == received_data[50]:
                unpacked_data = struct.unpack('f'*6, received_data[2:26])
                print(f"Position X: {unpacked_data[0]} , Velocity X: {unpacked_data[3]}")
                # print("Position Y:", unpacked_data[1])
                # print("Position Angular:", unpacked_data[2])
                # print("Velocity Y:", unpacked_data[4])
                # print("Velocity Angular:", unpacked_data[5])
            else:
                print("Checksum mismatch")
        else:
            print("Header mismatch")

def main():
    """Main function to control omni-wheel car."""
    try:
        while True:
            for i in range(10):
                start_time = time.time()
                while time.time() - start_time < 10:
                    send_velocity_command(i * 0.1)
                    read_serial_data()
                    time.sleep(0.1)
    except KeyboardInterrupt:
        print("Program stopped by user")
    finally:
        serial_conn.close()

if __name__ == "__main__":
    main()
