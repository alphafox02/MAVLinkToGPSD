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
from pymavlink import mavutil

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

async def get_gps_time(system):
    """Fetch GPS time from telemetry, fallback to SYSTEM_TIME if GPS is unavailable."""
    logging.info("Fetching GPS time from telemetry...")
    async for gps_info in system.telemetry.unix_epoch_time():
        if gps_info.time_us > 0:
            gps_time_seconds = gps_info.time_us / 1e6  # Convert microseconds to seconds
            logging.info(f"GPS time received: {gps_time_seconds}")
            return gps_time_seconds
        else:
            logging.warning("No GPS time available from telemetry.")
            return None

def set_system_time_using_mavlink(mavlink_connection):
    """Set system time using the MAVLink SYSTEM_TIME message."""
    logging.info("Using SYSTEM_TIME from MAVLink to update system time...")
    msg = mavlink_connection.recv_match(type='SYSTEM_TIME', blocking=True)
    if msg and msg.time_unix_usec > 0:
        gps_time_seconds = msg.time_unix_usec / 1e6
        update_system_time(gps_time_seconds)
        logging.info(f"System time set to: {gps_time_seconds}")
        return gps_time_seconds
    else:
        logging.warning("No valid SYSTEM_TIME received, falling back to system time.")
        return time.time()

def update_system_time(gps_time_seconds):
    """Update system time on the host."""
    current_time_sec = int(gps_time_seconds)
    current_time_nsec = int((gps_time_seconds - current_time_sec) * 1e9)
    
    class Timespec(ctypes.Structure):
        _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]

    clock_settime = ctypes.CDLL('libc.so.6').clock_settime
    clock_settime.argtypes = [ctypes.c_int, ctypes.POINTER(Timespec)]
    CLOCK_REALTIME = 0

    timespec = Timespec(tv_sec=current_time_sec, tv_nsec=current_time_nsec)
    if clock_settime(CLOCK_REALTIME, ctypes.byref(timespec)) == 0:
        logging.info(f"System time updated to {time.ctime(current_time_sec)}")
    else:
        logging.error("Failed to set system time. Are you running as root?")

async def handle_gps_time(system, mavlink_connection):
    """Manage GPS time by attempting to use telemetry first, then fallback to SYSTEM_TIME."""
    gps_time = await get_gps_time(system)
    if gps_time is None:
        gps_time = set_system_time_using_mavlink(mavlink_connection)
    return gps_time

def mavlink_to_gpsd_json(lat, lon, alt, speed, track, fix_type, timestamp):
    """Convert MAVSDK GPS data to a gpsd-style JSON TPV report."""
    mode = 3 if fix_type == 3 else 2 if fix_type == 2 else 1
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

async def run_server(system, mavlink_connection):
    """Run gpsd server and fetch GPS time."""
    while True:
        gps_time = await handle_gps_time(system, mavlink_connection)
        logging.info(f"Current GPS/System time: {gps_time}")
        await asyncio.sleep(10)

async def main():
    system = System()
    await system.connect(system_address="udp://127.0.0.1:14569")
    
    logging.info("Connected to MAVLink")

    mavlink_connection = mavutil.mavlink_connection('udp:127.0.0.1:14569')

    await run_server(system, mavlink_connection)

if __name__ == "__main__":
    asyncio.run(main())
