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
        ntp_time_frac = int((current_time % 1) * (2**32))  # Fraction of a second

        # Create the NTP response packet (properly packed)
        response = struct.pack(
            '!B B B b 11I',
            0b00100011,  # Leap Indicator (0), Version Number (4), Mode (3 for server)
            1,           # Stratum (secondary server)
            4,           # Poll interval (log2 seconds between messages)
            -6,          # Precision (log2 seconds)
            0,           # Root Delay (32-bit)
            0,           # Root Dispersion (32-bit)
            0xDEADBEEF,  # Reference ID (mocked, can be changed)
            ntp_time,    # Reference Timestamp seconds
            ntp_time_frac,  # Reference Timestamp fraction
            ntp_time,    # Originate Timestamp seconds
            ntp_time_frac,  # Originate Timestamp fraction
            ntp_time,    # Receive Timestamp seconds
            ntp_time_frac,  # Receive Timestamp fraction
            ntp_time,    # Transmit Timestamp seconds
            ntp_time_frac  # Transmit Timestamp fraction
        )

        # Send the response back to the client (Chrony)
        server_socket.sendto(response, address)

if __name__ == "__main__":
    ntp_server()
