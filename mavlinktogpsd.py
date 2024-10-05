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
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("/var/log/mavlink_gpsd.log"),
        logging.StreamHandler()
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
    current_time_sec = int(time_seconds)
    current_time_nsec = int((time_seconds - current_time_sec) * 1e9)

    shm = ShmTime()
    shm.mode = 1
    shm.clockTimeSec = current_time_sec
    shm.clockTimeNsec = current_time_nsec
    shm.receiveTimeSec = current_time_sec
    shm.receiveTimeNsec = current_time_nsec
    shm.valid = 1

    create_shm_file()

    try:
        with open(SHM_KEY, 'wb') as shm_file:
            shm_file.write(bytearray(shm))
            logging.debug(f"Updated SHM file with {source} time: {SHM_KEY}")
    except PermissionError:
        logging.error(f"Permission error: Unable to write to {SHM_KEY}. Ensure the script has the proper permissions.")

def clean_up_shm():
    if os.path.exists(SHM_KEY):
        try:
            subprocess.run(['sudo', 'rm', SHM_KEY], check=True)
            logging.info(f"Shared memory file {SHM_KEY} has been removed.")
        except subprocess.CalledProcessError as e:
            logging.error(f"Failed to remove SHM file: {e}")
    else:
        logging.debug(f"Shared memory file {SHM_KEY} does not exist.")

def mavlink_to_gpsd_json(lat, lon, alt, speed, track, fix_type, gps_time_seconds, use_shm=False):
    mode = 3 if fix_type == 3 else 2 if fix_type == 2 else 1

    gps_time = time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime(gps_time_seconds))
    
    if use_shm:
        update_shm_time(gps_time_seconds, source="GPS")
    
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
    sky_report = {
        "class": "SKY",
        "device": "/dev/mavlink",
        "satellites": []
    }
    
    for i in range(8):
        satellite = {
            "PRN": i + 1,
            "el": 45 + i,
            "az": (180 + i * 30) % 360,
            "ss": 40 + i,
            "used": i < 5
        }
        sky_report["satellites"].append(satellite)
    
    return json.dumps(sky_report) + "\n"

def start_gpsd_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('localhost', 2947))
    server_socket.listen(5)
    logging.info("Simulated gpsd running on port 2947. Waiting for clients...")
    return server_socket

def handle_client_connection(client_socket, gps_time_seconds, lat, lon, alt, speed, track, fix_type):
    try:
        client_socket.sendall(b'{"class":"VERSION","release":"3.20","rev":"3.20","proto_major":3,"proto_minor":14}\n')
        client_socket.sendall(b'{"class":"DEVICES","devices":[{"class":"DEVICE","path":"/dev/mavlink","activated":"2022-10-10T00:00:00.000Z","flags":1}]}\n')

        gpsd_report = mavlink_to_gpsd_json(lat, lon, alt, speed, track, fix_type, gps_time_seconds)
        client_socket.sendall(gpsd_report.encode('ascii'))
        logging.debug(f"Sent TPV: {gpsd_report.strip()}")

        sky_report = generate_sky_report()
        client_socket.sendall(sky_report.encode('ascii'))
        logging.debug(f"Sent SKY: {sky_report.strip()}")

    except (BrokenPipeError, ConnectionResetError):
        logging.warning("Client disconnected unexpectedly.")
    except KeyboardInterrupt:
        logging.info("Server interrupted by user.")
    finally:
        client_socket.close()

def poll_mavlink_for_gps(use_shm):
    gps_time_seconds = None
    while True:
        try:
            # Listen for SYSTEM_TIME for accurate GPS time
            system_time_msg = mavlink_connection.recv_match(type='SYSTEM_TIME', blocking=True)
            if system_time_msg and system_time_msg.time_unix_usec > 0:
                gps_time_seconds = system_time_msg.time_unix_usec / 1e6

            # Get the latest GPS_RAW_INT message for lat/lon
            gps_msg = mavlink_connection.recv_match(type='GPS_RAW_INT', blocking=True)
            if gps_msg and gps_time_seconds:
                lat = gps_msg.lat / 1e7
                lon = gps_msg.lon / 1e7
                alt = gps_msg.alt / 1000.0
                speed = gps_msg.vel / 100
                track = gps_msg.cog / 100
                fix_type = gps_msg.fix_type

                if use_shm:
                    update_shm_time(gps_time_seconds, source="GPS")

            time.sleep(1)

        except KeyboardInterrupt:
            logging.info("MAVLink polling interrupted by user.")
            break

def run_server(use_shm):
    polling_thread = threading.Thread(target=poll_mavlink_for_gps, args=(use_shm,))
    polling_thread.start()

    try:
        server_socket = start_gpsd_server()

        while True:
            try:
                client_socket, client_addr = server_socket.accept()
                logging.info(f"Client {client_addr} connected!")
                # Assume gps_time_seconds, lat, lon, etc. have been set from the MAVLink polling
                client_thread = threading.Thread(target=handle_client_connection, args=(client_socket,))
                client_thread.start()
            except KeyboardInterrupt:
                logging.info("Server interrupted by user.")
                break
    finally:
        server_socket.close()
        polling_thread.join()
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
        check_sudo()

    # MAVLink connection
    connection_string = 'udp:127.0.0.1:14569'
    mavlink_connection = mavutil.mavlink_connection(connection_string)

    mavlink_connection.wait_heartbeat()
    logging.info("Heartbeat received from MAVLink endpoint")

    run_server(args.use_shm)
