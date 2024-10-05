import socket
import struct
import time
from pymavlink import mavutil

# MAVLink connection setup
connection_string = 'udp:127.0.0.1:14570'
mavlink_connection = mavutil.mavlink_connection(connection_string)

# Function to get time from MAVLink SYSTEM_TIME message
def get_mavlink_time():
    msg = mavlink_connection.recv_match(type='SYSTEM_TIME', blocking=True)
    if msg:
        return msg.time_unix_usec / 1e6  # Convert from microseconds to seconds
    else:
        return time.time()  # Fallback to system time if MAVLink time is not available

# Simple NTP server implementation
def ntp_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(('0.0.0.0', 123))  # NTP runs on port 123

    print("NTP server is running on port 123...")

    while True:
        # Wait for an NTP request
        data, address = server_socket.recvfrom(1024)
        print(f"Received request from {address}")

        # Get the current time from MAVLink or system time
        current_time = get_mavlink_time()

        # Convert the current time to NTP format (seconds since 1900)
        ntp_time = int((current_time + 2208988800))  # 1900-based epoch

        # Create the NTP response packet (corrected to pack all 15 items)
        response = struct.pack(
            '!B B B b 11I',
            0b00100011,  # Leap Indicator, Version Number, Mode
            1,           # Stratum (secondary server)
            0,           # Poll interval (log2 seconds between messages)
            -6,          # Precision (log2 seconds)
            0, 0,        # Root Delay & Root Dispersion (not used here)
            0xDEADBEEF,  # Reference ID (mocked)
            ntp_time,    # Reference Timestamp
            0,           # Reference Timestamp (fraction)
            ntp_time,    # Originate Timestamp (time at client)
            0,           # Originate Timestamp (fraction)
            ntp_time,    # Receive Timestamp (time at server when received)
            0,           # Receive Timestamp (fraction)
            ntp_time,    # Transmit Timestamp (time at server when sent)
            0            # Transmit Timestamp (fraction)
        )

        # Send the response back to the client
        server_socket.sendto(response, address)

if __name__ == "__main__":
    ntp_server()
