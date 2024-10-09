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

# Setup rotating logging
log_handler = RotatingFileHandler("/var/log/mavlink_gpsd.log", maxBytes=5 * 1024 * 1024, backupCount=3)
logging.basicConfig(
    level=logging.INFO,  # Default to INFO; DEBUG will log more details if needed
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        log_handler,  # Log to file with rotation
        logging.StreamHandler()  # Log to stdout for systemd to capture
    ]
)

async def get_gps_time_from_mavsdk(system):
    """Fetch GPS time using MAVSDK telemetry with retry logic."""
    retries = 5
    while retries > 0:
        try:
            logging.info("Fetching GPS time from MAVSDK...")
            async for gps_time in system.telemetry.unix_epoch_time():
                gps_time_seconds = gps_time  # This gives us the correct Unix time in seconds
                logging.info(f"Received GPS Unix time: {gps_time_seconds}")
                return gps_time_seconds
        except grpc._channel._MultiThreadedRendezvous as e:
            logging.error(f"GRPC error: {e}, retrying in 5 seconds...")
            retries -= 1
            await asyncio.sleep(5)
    logging.error("Failed to get GPS time after several attempts.")
    return None

async def set_system_time_from_mavsdk(system):
    """Set system time using the MAVSDK telemetry Unix time."""
    while True:
        gps_time_seconds = await get_gps_time_from_mavsdk(system)
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

        await asyncio.sleep(60)  # Update system time every 60 seconds

def mavlink_to_gpsd_json(lat, lon, alt, speed, track, fix_type, timestamp):
    """Convert MAVSDK GPS data to a gpsd-style JSON TPV report."""
    mode = 3 if fix_type == 3 else 2 if fix_type == 2 else 1  # 3D, 2D, or no fix

    # Convert time to UTC
    gps_time = time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime(timestamp))
    logging.debug(f"GPS time: {gps_time}")

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

def handle_client_connection(client_socket, gps_time_seconds):
    """Handles communication with a connected client."""
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

        # Use the GPS time fetched via MAVSDK
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

async def run_server(system, update_system_time):
    """Runs the simulated gpsd server and handles multiple client connections."""
    if update_system_time:
        # Start system time updating in a separate thread
        time_update_task = asyncio.create_task(set_system_time_from_mavsdk(system))

    server_socket = start_gpsd_server()

    try:
        while True:
            client_socket, client_addr = server_socket.accept()
            logging.info(f"Client {client_addr} connected!")

            # Get GPS time for this connection
            gps_time_seconds = await get_gps_time_from_mavsdk(system)

            client_thread = threading.Thread(target=handle_client_connection, args=(client_socket, gps_time_seconds))
            client_thread.start()

    except KeyboardInterrupt:
        logging.info("Server interrupted by user.")
    finally:
        server_socket.close()
        if update_system_time:
            await time_update_task

async def main(update_system_time):
    # MAVSDK connection setup
    system = System()
    await system.connect(system_address="udp://127.0.0.1:14540")  # Replace with your MAVLink endpoint

    logging.info("Connected to MAVLink endpoint")

    # Start the gpsd server and optionally update system time
    await run_server(system, update_system_time)

# Command-line argument parsing
def parse_arguments():
    parser = argparse.ArgumentParser(description="Simulated gpsd with optional system time update.")
    parser.add_argument('--update-system-time', action='store_true', help="Enable system time updates from MAVLink.")
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_arguments()

    # Run the async main loop
    asyncio.run(main(args.update_system_time))
