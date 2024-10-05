import time
import os
import ctypes
from pymavlink import mavutil

# Connect to MAVLink (update the connection string if needed)
connection_string = 'udp:127.0.0.1:14569'
mavlink_connection = mavutil.mavlink_connection(connection_string)

# Function to set system time using the MAVLink SYSTEM_TIME message
def set_system_time_from_mavlink():
    print("Waiting for MAVLink SYSTEM_TIME message...")
    while True:
        try:
            msg = mavlink_connection.recv_match(type='SYSTEM_TIME', blocking=True)
            if msg:
                # Extract UNIX time in seconds (MAVLink provides time in microseconds)
                gps_time_seconds = msg.time_unix_usec / 1e6

                # Convert GPS time to the system time format (seconds and nanoseconds)
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
                    print(f"System time successfully updated to {time.ctime(current_time_sec)}")
                else:
                    print("Failed to set system time. Are you running as root?")

                # Sleep for a while before updating again (e.g., 60 seconds)
                time.sleep(60)

        except KeyboardInterrupt:
            print("System time update interrupted by user.")
            break

if __name__ == "__main__":
    set_system_time_from_mavlink()
