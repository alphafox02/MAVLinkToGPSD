import socket
import time
from pymavlink import mavutil
import json

# MAVLink connection
connection_string = 'udp:127.0.0.1:14550'
mavlink_connection = mavutil.mavlink_connection(connection_string)

# Wait for a heartbeat before requesting data
mavlink_connection.wait_heartbeat()
print("Heartbeat received from MAVLink endpoint")

def mavlink_to_gpsd_json(lat, lon, alt):
    """Convert MAVLink GPS data to a gpsd-style JSON TPV report."""
    report = {
        "class": "TPV",
        "device": "/dev/mavlink",
        "mode": 3,  # 3D Fix
        "lat": lat,  # Latitude
        "lon": lon,  # Longitude
        "alt": alt,  # Altitude in meters
        "altHAE": alt,  # Height above ellipsoid (same as alt for now)
        "speed": 0.0,  # Simulate speed
        "climb": 0.0,  # Simulate climb rate
        "track": 0.0,  # Simulate track (heading)
        "ept": 0.005,  # Estimated precision of time
        "epx": 5.0,  # Estimated precision of latitude
        "epy": 5.0,  # Estimated precision of longitude
        "epv": 5.0  # Estimated precision of altitude
    }
    return json.dumps(report) + "\n"


def start_gpsd_server():
    """Starts a simulated gpsd server on localhost:2947."""
    # Create a TCP socket to simulate gpsd
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('localhost', 2947))  # gpsd default port
    server_socket.listen(1)

    print("Simulated gpsd running on port 2947. Waiting for clients...")

    return server_socket


def handle_client_connection(client_socket):
    """Handles communication with a connected client."""
    try:
        # Send gpsd handshake messages
        client_socket.sendall(b'{"class":"VERSION","release":"3.20","rev":"3.20","proto_major":3,"proto_minor":14}\n')
        client_socket.sendall(b'{"class":"DEVICES","devices":[{"class":"DEVICE","path":"/dev/mavlink","activated":"2022-10-10T00:00:00.000Z","flags":1}]}\n')

        while True:
            # Receive MAVLink GPS data
            msg = mavlink_connection.recv_match(type=['GPS_RAW_INT', 'GLOBAL_POSITION_INT'], blocking=True)
            if msg:
                if msg.get_type() == 'GPS_RAW_INT':
                    lat = msg.lat / 1e7  # Convert to degrees
                    lon = msg.lon / 1e7  # Convert to degrees
                    alt = msg.alt / 1000.0  # Convert to meters
                elif msg.get_type() == 'GLOBAL_POSITION_INT':
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.alt / 1000.0

                # Convert to GPSD-style TPV JSON report
                gpsd_report = mavlink_to_gpsd_json(lat, lon, alt)

                # Send TPV data to the connected client (e.g., gpspipe or Sparrow WiFi)
                client_socket.sendall(gpsd_report.encode('ascii'))
                print(f"Sent TPV: {gpsd_report.strip()}")

            # Wait a short period before sending the next update
            time.sleep(1)

    except (BrokenPipeError, ConnectionResetError):
        print("Client disconnected unexpectedly.")
    except KeyboardInterrupt:
        print("Server interrupted by user.")
    finally:
        client_socket.close()


def run_server():
    """Runs the simulated gpsd server and handles client connections."""
    server_socket = start_gpsd_server()

    while True:
        try:
            # Wait for a client to connect (e.g., gpspipe or Sparrow WiFi)
            client_socket, client_addr = server_socket.accept()
            print(f"Client {client_addr} connected!")

            # Handle the client connection
            handle_client_connection(client_socket)

        except KeyboardInterrupt:
            print("Server interrupted by user.")
            break
        finally:
            client_socket.close()

    server_socket.close()


# Start the server
run_server()
