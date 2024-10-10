import socket
import time
import json
import threading
import argparse
import os
import ctypes
import logging
from logging.handlers import RotatingFileHandler
from mavsdk import System
import asyncio
from datetime import datetime

# Setup rotating logging
log_handler = RotatingFileHandler("/var/log/mavlink_gpsd.log", maxBytes=5 * 1024 * 1024, backupCount=3)
logging.basicConfig(
    level=logging.INFO,  # Default to INFO; DEBUG will log more details if needed
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[log_handler, logging.StreamHandler()]  # Log to file with rotation and stdout
)

async def get_gps_time_from_telemetry(system):
    """Fetch GPS time from MAVSDK telemetry."""
    async for unix_epoch_time in system.telemetry.unix_epoch_time():
        gps_time_seconds = unix_epoch_time.time_us / 1e6  # Convert microseconds to seconds
        return gps_time_seconds

async def set_system_time_from_telemetry(system):
    """Set system time using MAVSDK telemetry Unix time."""
    while True:
        gps_time_seconds = await get_gps_time_from_telemetry(system)
        if gps_time_seconds:
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
        await asyncio.sleep(60)

# Function to convert MAVLink GPS data to a gpsd-style JSON TPV report
def mavlink_to_gpsd_json(lat, lon, alt, speed, track, fix_type, timestamp):
    mode = 3 if fix_type == 3 else 2 if fix_type == 2 else 1  # 3D, 2D, or no fix
    gps_time = time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime(timestamp))
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
    server_socket.bind(('localhost', 2947))  # gpsd default port
    server_socket.listen(5)
    logging.info("Simulated gpsd running on port 2947. Waiting for clients...")
    return server_socket

async def handle_client_connection(client_socket, gps_time_seconds):
    try:
        # Send gpsd handshake messages
        client_socket.sendall(b'{"class":"VERSION","release":"3.20","rev":"3.20","proto_major":3,"proto_minor":14}\n')
        client_socket.sendall(b'{"class":"DEVICES","devices":[{"class":"DEVICE","path":"/dev/mavlink","activated":"2022-10-10T00:00:00.000Z","flags":1}]}\n')

        lat = 37.7749  # Example lat/lon data (can be updated)
        lon = -122.4194
        alt = 500  # Example altitude
        speed = 30
        track = 90
        fix_type = 3  # Simulated 3D fix

        gpsd_report = mavlink_to_gpsd_json(lat, lon, alt, speed, track, fix_type, gps_time_seconds)
        client_socket.sendall(gpsd_report.encode('ascii'))

        sky_report = generate_sky_report()
        client_socket.sendall(sky_report.encode('ascii'))

    except (BrokenPipeError, ConnectionResetError):
        logging.warning("Client disconnected unexpectedly.")
    except KeyboardInterrupt:
        logging.info("Server interrupted by user.")
    finally:
        client_socket.close()

async def run_server(system, update_system_time):
    if update_system_time:
        time_update_task = asyncio.create_task(set_system_time_from_telemetry(system))

    server_socket = start_gpsd_server()

    try:
        while True:
            client_socket, client_addr = server_socket.accept()
            logging.info(f"Client {client_addr} connected!")
            gps_time_seconds = await get_gps_time_from_telemetry(system)
            client_thread = threading.Thread(target=handle_client_connection, args=(client_socket, gps_time_seconds))
            client_thread.start()

    except KeyboardInterrupt:
        logging.info("Server interrupted by user.")
    finally:
        server_socket.close()
        if update_system_time:
            await time_update_task

async def main(update_system_time):
    system = System()
    await system.connect(system_address="udp://127.0.0.1:14569")  # Change to your endpoint

    logging.info("Connected to MAVLink endpoint.")
    await run_server(system, update_system_time)

def parse_arguments():
    parser = argparse.ArgumentParser(description="Simulated gpsd with optional system time update.")
    parser.add_argument('--update-system-time', action='store_true', help="Enable system time updates from MAVLink.")
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_arguments()
    asyncio.run(main(args.update_system_time))
