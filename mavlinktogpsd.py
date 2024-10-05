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
SHM_KEY = "/dev/shm/chrony0"

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
        logging.debug(f"Shared memory file {SHM_KEY} already exists.")
    else:
        try:
            subprocess.run(['sudo', 'touch', SHM_KEY], check=True)
            subprocess.run(['sudo', 'chmod', '777', SHM_KEY], check=True)
            logging.info(f"Created and set permissions for SHM file: {SHM_KEY}")
        except subprocess.CalledProcessError as e:
            logging.error(f"Failed to create SHM file: {e}")

def update_shm_time(time_seconds, source="GPS"):
    """Write GPS time to shared memory in Chrony's SHM format."""
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
            logging.debug(f"Updated SHM file with {source} time: {SHM_KEY}")
    except PermissionError:
        logging.error(f"Permission error: Unable to write to {SHM_KEY}. Ensure the script has the proper permissions.")

def clean_up_shm():
    """Remove the SHM file on cleanup."""
    if os.path.exists(SHM_KEY):
        try:
            subprocess.run(['sudo', 'rm', SHM_KEY], check=True)
            logging.info(f"Shared memory file {SHM_KEY} has been removed.")
        except subprocess.CalledProcessError as e:
            logging.error(f"Failed to remove SHM file: {e}")
    else:
        logging.debug(f"Shared memory file {SHM_KEY} does not exist.")

def mavlink_to_gpsd_json(lat, lon, alt, speed, track, fix_type, timestamp, use_shm=False):
    """Convert MAVLink GPS data to a gpsd-style JSON TPV report."""
    mode = 3 if fix_type == 3 else 2 if fix_type == 2 else 1  # 3D, 2D, or no fix

    # Ensure the GPS time is converted correctly from microseconds to seconds and in UTC
    if timestamp:
        gps_time = time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime(timestamp / 1e6))  # Proper GPS UTC time
        gps_time_seconds = timestamp / 1e6  # Convert to seconds for SHM
        if use_shm:
            update_shm_time(gps_time_seconds, source="GPS")  # Write GPS time to SHM
        logging.debug(f"GPS time: {gps_time} (valid timestamp from GPS)")
    else:
        # Fall back to system time if GPS time is not available
        gps_time = time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime())
        logging.warning(f"GPS timestamp missing, using system time: {gps_time}")

    report = {
        "class": "TPV",
        "device": "/dev/mavlink",
        "time": gps_time,  # Correct GPS/system time
        "mode": mode,  # GPS mode (2D or 3D fix)
        "lat": lat,  # Latitude
        "lon": lon,  # Longitude
        "alt": alt,  # Altitude in meters
        "altHAE": alt,  # Height above ellipsoid (same as alt for now)
        "speed": speed,  # Speed in m/s
        "track": track,  # Heading in degrees
        "ept": 0.005,  # Estimated precision of time
        "epx": 5.0,  # Estimated precision of latitude
        "epy": 5.0,  # Estimated precision of longitude
        "epv": 5.0  # Estimated precision of altitude
    }

    return json.dumps(report) + "\n"

def generate_sky_report():
    """Generate a GPSD SKY report with simulated satellite data."""
    sky_report = {
        "class": "SKY",
        "device": "/dev/mavlink",
        "satellites": []
    }
    
    # Simulate 8 satellites in view
    for i in range(8):
        satellite = {
            "PRN": i + 1,  # Satellite ID
            "el": 45 + i,  # Elevation in degrees (simulated)
            "az": (180 + i * 30) % 360,  # Azimuth in degrees (simulated)
            "ss": 40 + i,  # Signal strength (simulated)
            "used": i < 5  # Only use the first 5 satellites for the fix
        }
        sky_report["satellites"].append(satellite)
    
    return json.dumps(sky_report) + "\n"

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
            msg = mavlink_connection.recv_match(type='GPS_RAW_INT', blocking=True)
            if msg:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.alt / 1000.0
                speed = msg.vel / 100
                track = msg.cog / 100
                fix_type = msg.fix_type
                timestamp = msg.time_usec  # GPS time in microseconds

                # Create TPV JSON report for gpsd clients
                gpsd_report = mavlink_to_gpsd_json(lat, lon, alt, speed, track, fix_type, timestamp)
                client_socket.sendall(gpsd_report.encode('ascii'))
                logging.debug(f"Sent TPV report to client: {gpsd_report.strip()}")

                # Also send simulated SKY data (satellites in view)
                sky_report = generate_sky_report()
                client_socket.sendall(sky_report.encode('ascii'))
                logging.debug(f"Sent SKY report to client: {sky_report.strip()}")

            time.sleep(1)

    except (BrokenPipeError, ConnectionResetError):
        logging.warning("Client disconnected unexpectedly.")
    except KeyboardInterrupt:
        logging.info("Server interrupted by user.")
    finally:
        client_socket.close()

def poll_mavlink_for_gps(use_shm):
    """Poll MAVLink endpoint for GPS data and update SHM regularly regardless of client connections."""
    while True:
        try:
            msg = mavlink_connection.recv_match(type='GPS_RAW_INT', blocking=True)
            if msg:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.alt / 1000.0
                speed = msg.vel / 100
                track = msg.cog / 100
                fix_type = msg.fix_type
                timestamp = msg.time_usec  # GPS time in microseconds

                if use_shm:
                    gps_time_seconds = timestamp / 1e6  # Convert to seconds for SHM
                    update_shm_time(gps_time_seconds, source="GPS")

            time.sleep(1)

        except KeyboardInterrupt:
            logging.info("MAVLink polling interrupted by user.")
            break

def run_server(use_shm):
    """Runs the simulated gpsd server and handles multiple client connections."""
    # Start MAVLink polling in a separate thread to ensure SHM updates happen regardless of clients
    polling_thread = threading.Thread(target=poll_mavlink_for_gps, args=(use_shm,))
    polling_thread.start()

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
        polling_thread.join()  # Ensure the MAVLink polling thread terminates correctly
        if use_shm:
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

    # Start the gpsd server with optional shared memory support
    run_server(args.use_shm)
