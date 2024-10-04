import socket
import time
import json
import threading
import argparse
import os
import ctypes
from pymavlink import mavutil

# Create the shared memory key and attach to it (only if --use-shm is specified)
SHM_KEY = "/dev/shm/chrony_shm"  # Shared memory key

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

def update_shm_time(time_seconds):
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

    with open(SHM_KEY, 'wb') as shm_file:
        shm_file.write(bytearray(shm))

def clean_up_shm():
    """Clean up shared memory (remove SHM file)."""
    if os.path.exists(SHM_KEY):
        os.remove(SHM_KEY)
        print(f"Shared memory at {SHM_KEY} has been cleaned up.")

def mavlink_to_gpsd_json(lat, lon, alt, speed, track, fix_type, timestamp, use_shm=False):
    """Convert MAVLink GPS data to a gpsd-style JSON TPV report."""
    mode = 3 if fix_type == 3 else 2 if fix_type == 2 else 1  # 3D, 2D, or no fix
    
    if timestamp:
        gps_time = time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime(timestamp / 1e6))
        gps_time_seconds = timestamp / 1e6  # Convert to seconds
        if use_shm:
            update_shm_time(gps_time_seconds)  # Write time to shared memory for Chrony
    else:
        gps_time = time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime())
        print(f"Warning: Using system time {gps_time} due to missing GPS time.")

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
    """Generate a GPSD SKY report based on real satellite data from GPS_STATUS."""
    
    # Try to receive the GPS_STATUS message from MAVLink
    msg = mavlink_connection.recv_match(type='GPS_STATUS', blocking=False)
    
    if not msg:
        # If no GPS_STATUS message is available, return an empty SKY report
        return json.dumps({
            "class": "SKY",
            "device": "/dev/mavlink",
            "satellites": []
        }) + "\n"
    
    # Parse the GPS_STATUS message
    satellites_visible = msg.satellites_visible
    satellite_prn = msg.satellite_prn
    satellite_used = msg.satellite_used
    satellite_elevation = msg.satellite_elevation
    satellite_azimuth = msg.satellite_azimuth
    satellite_snr = msg.satellite_snr
    
    sky_report = {
        "class": "SKY",
        "device": "/dev/mavlink",
        "satellites": []
    }
    
    # Loop through visible satellites and add their information to the SKY report
    for i in range(satellites_visible):
        satellite = {
            "PRN": satellite_prn[i],  # Satellite ID
            "el": satellite_elevation[i],  # Elevation in degrees
            "az": int((satellite_azimuth[i] * 360) / 255),  # Convert azimuth to degrees
            "ss": satellite_snr[i],  # Signal strength (SNR) in dB
            "used": satellite_used[i] == 1  # True if the satellite is used for localization
        }
        sky_report["satellites"].append(satellite)
    
    return json.dumps(sky_report) + "\n"

def start_gpsd_server():
    """Starts a simulated gpsd server on localhost:2947."""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('localhost', 2947))  # gpsd default port
    server_socket.listen(5)  # Allow up to 5 concurrent connections
    print("Simulated gpsd running on port 2947. Waiting for clients...")
    return server_socket

def handle_client_connection(client_socket, use_shm):
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
                timestamp = msg.time_usec

                gpsd_report = mavlink_to_gpsd_json(lat, lon, alt, speed, track, fix_type, timestamp, use_shm)
                client_socket.sendall(gpsd_report.encode('ascii'))
                print(f"Sent TPV: {gpsd_report.strip()}")

                sky_report = generate_sky_report()
                client_socket.sendall(sky_report.encode('ascii'))
                print(f"Sent SKY: {sky_report.strip()}")

            time.sleep(1)

    except (BrokenPipeError, ConnectionResetError):
        print("Client disconnected unexpectedly.")
    except KeyboardInterrupt:
        print("Server interrupted by user.")
    finally:
        client_socket.close()

def run_server(use_shm):
    """Runs the simulated gpsd server and handles multiple client connections."""
    try:
        server_socket = start_gpsd_server()

        while True:
            try:
                client_socket, client_addr = server_socket.accept()
                print(f"Client {client_addr} connected!")
                client_thread = threading.Thread(target=handle_client_connection, args=(client_socket, use_shm))
                client_thread.start()
            except KeyboardInterrupt:
                print("Server interrupted by user.")
                break
    finally:
        server_socket.close()
        if use_shm:
            clean_up_shm()

# Command-line argument parsing
def parse_arguments():
    parser = argparse.ArgumentParser(description="Simulated gpsd with optional SHM support.")
    parser.add_argument('--use-shm', action='store_true', help="Enable shared memory (SHM) for Chrony.")
    return parser.parse_args()

def check_sudo():
    if os.geteuid() != 0:
        print("This script must be run with sudo to enable shared memory (SHM).")
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
    print("Heartbeat received from MAVLink endpoint")

    # Start the gpsd server with optional shared memory support
    run_server(args.use_shm)
