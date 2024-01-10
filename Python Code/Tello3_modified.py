import threading 
import socket
import sys
import time
import serial  # Import the pyserial library

host = '...'
port = 9000
locaddr = (host, port) 

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tello_address = ('192.168.10.1', 8889)
sock.bind(locaddr)

# Serial port configuration
uart_port = '...'
uart_baudrate = 9600
ser = serial.Serial(uart_port, uart_baudrate, timeout=1)

def recv():
    count = 0
    while True: 
        try:
            data, server = sock.recvfrom(1518)
            print(data.decode(encoding="utf-8"))
        except Exception:
            print('\nExit . . .\n')
            break

def read_uart():
    while True:
        uart_data = ser.readline().decode("utf-8").strip()
        if uart_data:
            print("Received from UART:", uart_data)
            # Send UART data to Tello drone
            msg = uart_data.encode(encoding="utf-8")
            sent = sock.sendto(msg, tello_address)

# Create a thread for receiving responses from the Tello drone
recv_thread = threading.Thread(target=recv)
recv_thread.start()

# Create a thread for reading commands from UART
uart_thread = threading.Thread(target=read_uart)
uart_thread.start()

try:
    print('\r\n\r\nTello Python3 Demo.\r\n')
    print('Tello: command takeoff land flip forward back left right \r\n       up down cw ccw speed speed?\r\n')
    print('end -- quit demo.\r\n')

    while True: 
        try:
            msg = input("")

            if not msg:
                break  

            if 'end' in msg:
                print('...')
                sock.close()  
                ser.close()
                break

            # Send data
            msg = msg.encode(encoding="utf-8") 
            sent = sock.sendto(msg, tello_address)
        except KeyboardInterrupt:
            print('\n . . .\n')
            sock.close()
            ser.close()
            break
finally:
    sock.close()
    ser.close()
