import socket
import time
import json
import threading
from pymavlink import mavutil

# MAVLink connection
connection_string = 'udp:127.0.0.1:14569'
mavlink_connection = mavutil.mavlink_connection(connection_string)

# Wait for a heartbeat before requesting data
mavlink_connection.wait_heartbeat()
print("Heartbeat received from MAVLink endpoint")

def mavlink_to_gpsd_json(lat, lon, alt, speed, track, fix_type, timestamp):
    """Convert MAVLink GPS data to a gpsd-style JSON TPV report."""
    # Convert MAVLink fix type to gpsd mode
    mode = 3 if fix_type == 3 else 2 if fix_type == 2 else 1  # 3D, 2D, or no fix

    # Use GPS time from MAVLink if available, fallback to system time with a warning
    if timestamp:
        gps_time = time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime(timestamp / 1e6))
        time_source = "GPS"
    else:
        gps_time = time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime())
        time_source = "System"
        print(f"Warning: Using system time {gps_time} due to missing GPS time.")

    report = {
        "class": "TPV",
        "device": "/dev/mavlink",
        "time": gps_time,  # Properly formatted GPS/system time
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
    """Generate a simulated GPSD SKY report with satellite info."""
    sky_report = {
        "class": "SKY",
        "device": "/dev/mavlink",
        "satellites": []
    }
    
    # Simulate 8 satellites in view
    for i in range(8):
        satellite = {
            "PRN": i + 1,  # Satellite ID
            "el": 45 + i,  # Elevation (degrees)
            "az": (180 + i * 30) % 360,  # Azimuth (degrees)
            "ss": 40 + i,  # Signal strength (dBHz)
            "used": True if i < 5 else False  # Only use the first 5 satellites for the fix
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

def handle_client_connection(client_socket):
    """Handles communication with a connected client."""
    try:
        # Send gpsd handshake messages
        client_socket.sendall(b'{"class":"VERSION","release":"3.20","rev":"3.20","proto_major":3,"proto_minor":14}\n')
        client_socket.sendall(b'{"class":"DEVICES","devices":[{"class":"DEVICE","path":"/dev/mavlink","activated":"2022-10-10T00:00:00.000Z","flags":1}]}\n')

        while True:
            # Receive MAVLink GPS_RAW_INT data only
            msg = mavlink_connection.recv_match(type='GPS_RAW_INT', blocking=True)
            if msg:
                lat = msg.lat / 1e7  # Convert to degrees
                lon = msg.lon / 1e7  # Convert to degrees
                alt = msg.alt / 1000.0  # Convert to meters
                speed = msg.vel / 100  # Convert from cm/s to m/s
                track = msg.cog / 100  # Convert from centi-degrees to degrees
                fix_type = msg.fix_type  # 1 = No fix, 2 = 2D fix, 3 = 3D fix
                timestamp = msg.time_usec  # GPS time in microseconds

                # Convert to GPSD-style TPV JSON report
                gpsd_report = mavlink_to_gpsd_json(lat, lon, alt, speed, track, fix_type, timestamp)

                # Send TPV data to the connected client
                client_socket.sendall(gpsd_report.encode('ascii'))
                print(f"Sent TPV: {gpsd_report.strip()}")

                # Send simulated SKY (satellite) data
                sky_report = generate_sky_report()
                client_socket.sendall(sky_report.encode('ascii'))
                print(f"Sent SKY: {sky_report.strip()}")

            time.sleep(1)  # Wait a short period before sending the next update

    except (BrokenPipeError, ConnectionResetError):
        print("Client disconnected unexpectedly.")
    except KeyboardInterrupt:
        print("Server interrupted by user.")
    finally:
        client_socket.close()

def run_server():
    """Runs the simulated gpsd server and handles multiple client connections."""
    server_socket = start_gpsd_server()

    while True:
        try:
            client_socket, client_addr = server_socket.accept()
            print(f"Client {client_addr} connected!")
            # Handle the client connection in a new thread
            client_thread = threading.Thread(target=handle_client_connection, args=(client_socket,))
            client_thread.start()
        except KeyboardInterrupt:
            print("Server interrupted by user.")
            break

    server_socket.close()

# Start the server
run_server()
