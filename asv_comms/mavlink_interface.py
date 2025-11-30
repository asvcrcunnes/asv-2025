from pymavlink import mavutil
import threading
import time
import math

class MavlinkInterface:
    def __init__(self, connection_string, baudrate=115200):
        self.conn_str = connection_string
        self.baud = baudrate
        self.vehicle = None
        self.running = False
        
        # Shared State (Data Kapal)
        # Inilah data yang akan kita kirim ke Web & digunakan FSM
        self.state = {
            "connected": False,
            "armed": False,
            "mode": "UNKNOWN",
            "lat": 0.0,
            "lon": 0.0,
            "heading": 0.0,
            "cog": 0.0,
            "sog": 0.0,       # Speed over ground (m/s)
            "battery": 0.0,
            "local_x": 0.0,   # Posisi lokal Utara (meter) dari Home
            "local_y": 0.0,   # Posisi lokal Timur (meter) dari Home
            "roll": 0.0,
            "pitch": 0.0
        }
        
        self.lock = threading.Lock()

    def connect(self):
        print(f"[Mavlink] Connecting to {self.conn_str}...")
        try:
            self.vehicle = mavutil.mavlink_connection(self.conn_str, baud=self.baud)
            self.vehicle.wait_heartbeat(timeout=5)
            print("[Mavlink] Heartbeat received! Connected.")
            self.state["connected"] = True
            
            # Request Data Stream (Penting agar data update cepat)
            self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10) # 10Hz
            self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5) # 5Hz
            self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)           # 10Hz
            self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 5)             # 5Hz
            self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1)          # 1Hz
            
            return True
        except Exception as e:
            print(f"[Mavlink] Connection failed: {e}")
            return False

    def start(self):
        if not self.vehicle:
            if not self.connect():
                return
        
        self.running = True
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()
        print("[Mavlink] Listener thread started.")

    def _loop(self):
        while self.running:
            try:
                # Ambil pesan terbaru
                msg = self.vehicle.recv_match(blocking=True, timeout=1.0)
                
                if not msg:
                    continue
                
                type = msg.get_type()
                
                with self.lock:
                    if type == 'HEARTBEAT':
                        self.state["mode"] = mavutil.mode_string_v10(msg)
                        self.state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        
                    elif type == 'GLOBAL_POSITION_INT':
                        self.state["lat"] = msg.lat / 1e7
                        self.state["lon"] = msg.lon / 1e7
                        self.state["heading"] = msg.hdg / 100.0
                        
                    elif type == 'LOCAL_POSITION_NED':
                        # INI PENTING UNTUK PETA 25x25 WEB ANDA
                        self.state["local_x"] = msg.x  # Utara (meter)
                        self.state["local_y"] = msg.y  # Timur (meter)
                        
                    elif type == 'VFR_HUD':
                        self.state["sog"] = msg.groundspeed
                        self.state["cog"] = msg.heading # Kadang VFR_HUD heading lebih stabil utk COG
                        
                    elif type == 'SYS_STATUS':
                        self.state["battery"] = msg.battery_remaining
                        
                    elif type == 'ATTITUDE':
                        self.state["roll"] = math.degrees(msg.roll)
                        self.state["pitch"] = math.degrees(msg.pitch)

            except Exception as e:
                print(f"[Mavlink Loop Error] {e}")
                time.sleep(0.1)

    # --- FUNGSI KONTROL UNTUK FSM ---
    
    def set_mode(self, mode_name):
        """Mengubah mode (MANUAL, GUIDED, RTL, LOITER)"""
        # Mapping string ke ID mode ArduRover
        # Perlu referensi mode mapping ArduRover spesifik, tapi umumnya:
        mode_id = self.vehicle.mode_mapping().get(mode_name)
        if mode_id is None:
            print(f"[Mavlink] Unknown mode: {mode_name}")
            return
        
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"[Mavlink] Request mode change to: {mode_name}")

    def send_guided_velocity(self, vx, vy, vz=0):
        """
        Mengirim target kecepatan (meter/detik) di frame BODY kapal.
        vx: Maju (+), Mundur (-)
        vy: Kanan (+), Kiri (-) -> Untuk kapal skid steer/vectored
        vz: 0
        """
        # Type mask: Ignore position, accel, yaw rate. Only use Velocity + Yaw Angle
        # 0b0000111111000111
        type_mask = 0b0000111111000111
        
        self.vehicle.mav.set_position_target_local_ned_send(
            0, # time_boot_ms
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED, # Frame BODY (ikut arah hidung kapal)
            type_mask,
            0, 0, 0,        # x, y, z positions (ignored)
            vx, vy, vz,     # x, y, z velocity in m/s
            0, 0, 0,        # x, y, z acceleration (ignored)
            0, 0            # yaw, yaw_rate (ignored)
        )

    def request_message_interval(self, message_id, frequency_hz):
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, 1e6 / frequency_hz, 0, 0, 0, 0, 0
        )

    def get_state(self):
        with self.lock:
            return self.state.copy()