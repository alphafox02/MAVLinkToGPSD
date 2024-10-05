import socket
import time
import json
import threading
import argparse
import os
import ctypes
import subprocess
import logging
from pymavlink import mavutil

# Setup logging
logging.basicConfig(
    level=logging.INFO,  # Default to INFO; DEBUG will log more details if needed
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("/var/log/mavlink_gpsd.log"),  # Log to file
        logging.StreamHandler()  # Log to stdout for systemd to capture
    ]
)

# Path to the shared memory file
SHM_KEY = "/dev/shm/chrony_shm"

# Track the last update time to avoid redundant SHM updates
last_update_time = None

# Define the SHM structure
class ShmTime(ctypes.Structure):
    _fields_ = [("mode", ctypes.c_int),
                ("count", ctypes.c_uint),
                ("clockTimeSec", ctypes.c_int),
                ("clockTimeNsec", ctypes.c_int),
                ("receiveTimeSec", ctypes.c_int),
                ("receiveTimeNsec", ctypes.c_int),
                ("leap", ctypes.c_int),
                ("precision", ctypes.c_int),
                ("nsamples", ctypes.c_int),
                ("valid", ctypes.c_int)]

def create_shm_file():
    """Ensure that the SHM file exists and is writable."""
    if os.path.exists(SHM_KEY):
        logging.info(f"Shared memory file {SHM_KEY} already exists.")
    else:
        try:
            subprocess.run(['sudo', 'touch', SHM_KEY], check=True)
            subprocess.run(['sudo', 'chmod', '777', SHM_KEY], check=True)
            logging.info(f"Created and set permissions for SHM file: {SHM_KEY}")
        except subprocess.CalledProcessError as e:
            logging.error(f"Failed to create SHM file: {e}")

def update_shm_time(time_seconds, source="System"):
    """Write time (system or GPS) to shared memory in Chrony's SHM format."""
    global last_update_time

    # Only update SHM if time has changed significantly (to avoid redundant updates)
    if last_update_time is not None and abs(last_update_time - time_seconds) < 1:
        return  # No significant change in time; skip the update

    current_time_sec = int(time_seconds)
    current_time_nsec = int((time_seconds - current_time_sec) * 1e9)

    shm = ShmTime()
    shm.mode = 1  # Use GPS mode
    shm.clockTimeSec = current_time_sec
    shm.clockTimeNsec = current_time_nsec
    shm.receiveTimeSec = current_time_sec
    shm.receiveTimeNsec = current_time_nsec
    shm.valid = 1  # Time is valid

    # Ensure the SHM file exists
    create_shm_file()

    # Write to the SHM file
    try:
        with open(SHM_KEY, 'wb') as shm_file:
            shm_file.write(bytearray(shm))
            logging.info(f"Updated SHM file with {source} time: {time_seconds}")
        last_update_time = time_seconds  # Track last updated time
    except PermissionError:
        logging.error(f"Permission error: Unable to write to {SHM_KEY}. Ensure the script has the proper permissions.")

def mimic_gpsd_with_system_time():
    """Mimic gpsd by updating SHM with system time until GPS data is available."""
    while True:
        system_time = time.time()  # Get current system time
        update_shm_time(system_time, source="System")  # Write system time to SHM
        time.sleep(1)  # Update every second until GPS data is available

def poll_mavlink_for_gps():
    """Poll MAVLink endpoint for GPS data and update SHM with GPS time."""
    while True:
        msg = mavlink_connection.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            # Extract GPS data from MAVLink message
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0
            speed = msg.vel / 100
            track = msg.cog / 100
            fix_type = msg.fix_type
            timestamp = msg.time_usec  # Get GPS time in microseconds

            # Update SHM with GPS time
            gps_time_seconds = timestamp / 1e6  # Convert to seconds
            update_shm_time(gps_time_seconds, source="GPS")

            logging.info(f"Received GPS data: lat={lat}, lon={lon}, alt={alt}, time={gps_time_seconds}")
        time.sleep(1)  # Avoid busy-waiting

def clean_up_shm():
    """Remove the SHM file on cleanup."""
    if os.path.exists(SHM_KEY):
        try:
            subprocess.run(['sudo', 'rm', SHM_KEY], check=True)
            logging.info(f"Shared memory file {SHM_KEY} has been removed.")
        except subprocess.CalledProcessError as e:
            logging.error(f"Failed to remove SHM file: {e}")
    else:
        logging.info(f"Shared memory file {SHM_KEY} does not exist.")

def start_gpsd_server():
    """Starts a simulated gpsd server on localhost:2947."""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('localhost', 2947))  # gpsd default port
    server_socket.listen(5)  # Allow up to 5 concurrent connections
    logging.info("Simulated gpsd running on port 2947. Waiting for clients...")
    return server_socket

def handle_client_connection(client_socket):
    """Handles communication with a connected client."""
    try:
        # Send gpsd handshake messages
        client_socket.sendall(b'{"class":"VERSION","release":"3.20","rev":"3.20","proto_major":3,"proto_minor":14}\n')
        client_socket.sendall(b'{"class":"DEVICES","devices":[{"class":"DEVICE","path":"/dev/mavlink","activated":"2022-10-10T00:00:00.000Z","flags":1}]}\n')

        while True:
            # Handle client requests (if any)...
            time.sleep(1)
    except (BrokenPipeError, ConnectionResetError):
        logging.warning("Client disconnected unexpectedly.")
    except KeyboardInterrupt:
        logging.info("Server interrupted by user.")
    finally:
        client_socket.close()

def run_server():
    """Runs the simulated gpsd server and handles multiple client connections."""
    try:
        server_socket = start_gpsd_server()

        while True:
            try:
                client_socket, client_addr = server_socket.accept()
                logging.info(f"Client {client_addr} connected!")
                client_thread = threading.Thread(target=handle_client_connection, args=(client_socket,))
                client_thread.start()
            except KeyboardInterrupt:
                logging.info("Server interrupted by user.")
                break
    finally:
        server_socket.close()
        clean_up_shm()

# Command-line argument parsing
def parse_arguments():
    parser = argparse.ArgumentParser(description="Simulated gpsd with optional SHM support.")
    parser.add_argument('--use-shm', action='store_true', help="Enable shared memory (SHM) for Chrony.")
    return parser.parse_args()

def check_sudo():
    if os.geteuid() != 0:
        logging.error("This script must be run with sudo to enable shared memory (SHM).")
        exit(1)

# Main execution
if __name__ == "__main__":
    args = parse_arguments()

    if args.use_shm:
        check_sudo()  # Check if running with sudo when using shared memory

    # MAVLink connection
    connection_string = 'udp:127.0.0.1:14569'  # Replace with your MAVLink endpoint
    mavlink_connection = mavutil.mavlink_connection(connection_string)

    # Wait for a heartbeat before requesting data
    mavlink_connection.wait_heartbeat()
    logging.info("Heartbeat received from MAVLink endpoint")

    # Start thread to mimic gpsd with system time
    if args.use_shm:
        system_time_thread = threading.Thread(target=mimic_gpsd_with_system_time)
        system_time_thread.daemon = True
        system_time_thread.start()

    # Start thread to poll MAVLink for GPS data and update SHM
    gps_poll_thread = threading.Thread(target=poll_mavlink_for_gps)
    gps_poll_thread.daemon = True
    gps_poll_thread.start()

    # Run the GPSD server
    run_server()
