import serial
import socket

# Open a serial connection to the arduino
ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1)

# Open a socket connection over localhost to port 9988
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("localhost", 9988))
s.listen(1)

# Start an infinite loop to listen for packets from script_socket.py
while True:
    conn, addr = s.accept()
    # Get the data from the connection
    data = conn.recv(1024)
    conn.close()
    # Write the speed to arduino over serial
    ser.write(str(data))
    # Print the output to the terminal
    print(str(data))
    