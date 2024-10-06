import socket
import time
import json
import threading
import argparse
import os
import ctypes
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

# Function to set system time using the MAVLink SYSTEM_TIME message
def set_system_time_from_mavlink(mavlink_connection):
    """Set system time from MAVLink SYSTEM_TIME message."""
    logging.info("Waiting for MAVLink SYSTEM_TIME message to update system time...")
    while True:
        try:
            msg = mavlink_connection.recv_match(type='SYSTEM_TIME', blocking=True)
            if msg:
                # Extract UNIX time in seconds (MAVLink provides time in microseconds)
                gps_time_seconds = msg.time_unix_usec / 1e6

                # Convert GPS time to system time format (seconds and nanoseconds)
                current_time_sec = int(gps_time_seconds)
                current_time_nsec = int((gps_time_seconds - current_time_sec) * 1e9)

                # Create a timespec structure to hold the time values
                class Timespec(ctypes.Structure):
                    _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]

                # Update system time using clock_settime (requires root privileges)
                clock_settime = ctypes.CDLL('libc.so.6').clock_settime
                clock_settime.argtypes = [ctypes.c_int, ctypes.POINTER(Timespec)]
                CLOCK_REALTIME = 0  # Constant for system real-time clock

                timespec = Timespec(tv_sec=current_time_sec, tv_nsec=current_time_nsec)
                if clock_settime(CLOCK_REALTIME, ctypes.byref(timespec)) == 0:
                    logging.info(f"System time updated to {time.ctime(current_time_sec)}")
                else:
                    logging.error("Failed to set system time. Are you running as root?")
                
                time.sleep(60)  # Update system time every 60 seconds

        except KeyboardInterrupt:
            logging.info("System time update interrupted by user.")
            break

def mavlink_to_gpsd_json(lat, lon, alt, speed, track, fix_type, timestamp):
    """Convert MAVLink GPS data to a gpsd-style JSON TPV report."""
    mode = 3 if fix_type == 3 else 2 if fix_type == 2 else 1  # 3D, 2D, or no fix

    # Convert time to UTC
    gps_time = time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime(timestamp))
    logging.debug(f"GPS time from SYSTEM_TIME: {gps_time}")

    report = {
        "class": "TPV",
        "device": "/dev/mavlink",
        "time": gps_time,
        "mode": mode,
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "altHAE": alt,
        "speed": speed,
        "track": track,
        "ept": 0.005,
        "epx": 5.0,
        "epy": 5.0,
        "epv": 5.0
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

                # Fetch the SYSTEM_TIME message for accurate timestamp
                system_time_msg = mavlink_connection.recv_match(type='SYSTEM_TIME', blocking=False)
                if system_time_msg:
                    timestamp = system_time_msg.time_unix_usec / 1e6  # Convert from microseconds to seconds
                else:
                    timestamp = time.time()  # Fallback to system time

                gpsd_report = mavlink_to_gpsd_json(lat, lon, alt, speed, track, fix_type, timestamp)
                client_socket.sendall(gpsd_report.encode('ascii'))
                logging.debug(f"Sent TPV: {gpsd_report.strip()}")

                sky_report = generate_sky_report()
                client_socket.sendall(sky_report.encode('ascii'))
                logging.debug(f"Sent SKY: {sky_report.strip()}")

            time.sleep(1)

    except (BrokenPipeError, ConnectionResetError):
        logging.warning("Client disconnected unexpectedly.")
    except KeyboardInterrupt:
        logging.info("Server interrupted by user.")
    finally:
        client_socket.close()

def poll_mavlink_for_gps():
    """Poll MAVLink endpoint for SYSTEM_TIME and GPS_RAW_INT."""
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

                # Fetch the SYSTEM_TIME message for accurate timestamp
                system_time_msg = mavlink_connection.recv_match(type='SYSTEM_TIME', blocking=False)
                if system_time_msg:
                    timestamp = system_time_msg.time_unix_usec / 1e6  # Convert to seconds
                else:
                    timestamp = time.time()  # Fallback to system time

            time.sleep(1)

        except KeyboardInterrupt:
            logging.info("MAVLink polling interrupted by user.")
            break

def run_server(update_system_time):
    """Runs the simulated gpsd server and handles multiple client connections."""
    if update_system_time:
        # Start system time updating in a separate thread
        time_update_thread = threading.Thread(target=set_system_time_from_mavlink, args=(mavlink_connection,))
        time_update_thread.start()

    # Start MAVLink polling in a separate thread
    polling_thread = threading.Thread(target=poll_mavlink_for_gps)
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
        if update_system_time:
            time_update_thread.join()  # Ensure the time update thread terminates

# Command-line argument parsing
def parse_arguments():
    parser = argparse.ArgumentParser(description="Simulated gpsd with optional system time update.")
    parser.add_argument('--update-system-time', action='store_true', help="Enable system time updates from MAVLink.")
    return parser.parse_args()

# Main execution
if __name__ == "__main__":
    args = parse_arguments()

    # MAVLink connection
    connection_string = 'udp:127.0.0.1:14569'  # Replace with your MAVLink endpoint
    mavlink_connection = mavutil.mavlink_connection(connection_string)

    # Wait for a heartbeat before requesting data
    mavlink_connection.wait_heartbeat()
    logging.info("Heartbeat received from MAVLink endpoint")

    # Start the gpsd server and optionally update system time
    run_server(args.update_system_time)

