import threading
import time
import cv2
import numpy as np
import io
from PIL import Image
from datetime import datetime
import firebase_admin
from firebase_admin import credentials, db
import cloudinary
import cloudinary.uploader

class WebPublisher:
    def __init__(self, key_path, db_url, db_node, cloud_config):
        # 1. Setup Firebase
        if not firebase_admin._apps:
            cred = credentials.Certificate(key_path)
            firebase_admin.initialize_app(cred, {'databaseURL': db_url})
        self.ref = db.reference(db_node)
        
        # 2. Setup Cloudinary
        cloudinary.config(**cloud_config)
        
        # 3. State Internal
        self.telemetry = {}
        self.mission_log = {
            "preparation": "Done",
            "start": "Pending",
            "floating_ball": 0,
            "surface_imaging": "Pending",
            "underwater_imaging": "Pending",
            "finish": "Pending"
        }
        self.mission_images = {"surface": None, "underwater": None}
        self.track_id = "A" # Default lintasan A
        self.running = False
        self.lock = threading.Lock()

    def start(self):
        self.running = True
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()
        print("[Web] Publisher thread started.")

    def update_telemetry(self, mavlink_state):
        """Dipanggil oleh main.py setiap loop untuk update data sensor"""
        with self.lock:
            self.telemetry = mavlink_state

    def update_mission_status(self, key, status):
        """Update status misi (misal: 'surface_imaging': 'In Progress')"""
        with self.lock:
            self.mission_log[key] = status

    def upload_image(self, frame, mission_type, sensor_data):
        """Dipanggil saat FSM memutuskan untuk memotret"""
        print(f"[Web] Uploading {mission_type} image...")
        
        # Jalankan di thread terpisah agar tidak memblokir navigasi
        t = threading.Thread(target=self._upload_task, args=(frame, mission_type, sensor_data))
        t.start()

    def _upload_task(self, frame, mission_type, sensor_data):
        try:
            # 1. Gambar Geotag
            frame_tagged = self._draw_geotag(frame.copy(), sensor_data)
            
            # 2. Convert ke Buffer JPEG
            frame_rgb = cv2.cvtColor(frame_tagged, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            buffer = io.BytesIO()
            img.save(buffer, format="JPEG", quality=80)
            buffer.seek(0)
            
            # 3. Upload Cloudinary
            res = cloudinary.uploader.upload(buffer, folder="asv_lomba")
            url = res.get('secure_url')
            
            with self.lock:
                self.mission_images[mission_type] = url
                self.mission_log[f"{mission_type}_imaging"] = "Done"
            
            print(f"[Web] Upload {mission_type} success: {url}")
            
        except Exception as e:
            print(f"[Web] Upload failed: {e}")
            with self.lock:
                self.mission_log[f"{mission_type}_imaging"] = "Failed"

    def _draw_geotag(self, frame, data):
        # ... (Gunakan logika draw_geotag_on_image Anda yang sebelumnya di sini) ...
        # Untuk mempersingkat, saya asumsikan Anda copy-paste fungsi draw_text OpenCV di sini
        return frame

    def _loop(self):
        while self.running:
            try:
                # Siapkan Payload JSON sesuai Kontrak Data Bagian 2
                payload = {
                    "track_id": self.track_id,
                    "current_mission": "Autonomous Run",
                    "attitude": {
                        "sog": self.telemetry.get("sog", 0),
                        "cog": self.telemetry.get("cog", 0),
                        "heading": self.telemetry.get("heading", 0)
                    },
                    "gps_location": {
                        "lat": self.telemetry.get("lat", 0),
                        "lon": self.telemetry.get("lon", 0)
                    },
                    "local_position": {
                        # MAPPING PENTING: ArduRover North(X) -> Web Y, East(Y) -> Web X
                        "x": self.telemetry.get("local_y", 0), 
                        "y": self.telemetry.get("local_x", 0)
                    },
                    "indicators": {
                        "battery": self.telemetry.get("battery", 0),
                        "last_update": time.time()
                    },
                    "mission_images": self.mission_images,
                    "position_log": self.mission_log
                }
                
                # Kirim ke Firebase
                self.ref.set(payload)
                time.sleep(1.0) # Update 1 Hz
                
            except Exception as e:
                print(f"[Web Loop Error] {e}")
                time.sleep(2)