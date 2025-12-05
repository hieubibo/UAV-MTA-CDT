#!/usr/bin/env python3
from pymavlink import mavutil
import log
import flight_control
import time

def connect(f,port="/dev/ttyUSB0", baud=57600):
    f.write(f"Connecting to {port} @ {baud}...")
    m = mavutil.mavlink_connection(port, baud=baud)
    m.wait_heartbeat()
    f.write(f"✔ HEARTBEAT received: {m.target_system}, {m.target_component}")
    return m

if __name__ =="__main__":
    try:
        f = log.log_file()
        m = connect(f)
        log.request_message(m,65)
        log.request_message(m,33)
        while True:
            ch6, _ = log.stream_data(m,f)
            time.sleep(0.05)
            if ch6 == 2000:
                break
        flight_control.set_mode_guided(m,f)
        flight_control.arm(m,f)
        time.sleep(1.5)
        flight_control.takeoff(m,2.5,f)
        t0 = time.time()
        while time.time() - t0 < 20:
            _, alt_m = log.stream_data(m,f)
            if time.time() - t0 > 8:
                flight_control.send_body_velocity(m,0,0,0)
            time.sleep(0.05)

        flight_control.land(m,f)
        
    except:
        pass
    finally:
        flight_control.land(m,f)
        log.stop_stream(m,f)