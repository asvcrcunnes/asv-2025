# config.py

# --- KONEKSI ---
# MAVLINK_CONNECTION = '/dev/ttyACM0' # RPi ke Pixhawk via USB
MAVLINK_CONNECTION = 'COM3'         # Windows (untuk testing)
MAVLINK_BAUD = 115200

# --- KAMERA ---
# Pastikan Anda tahu ID masing-masing kamera di RPi (/dev/video0, 1, 2)
CAM_NAV_INDEX = 0        # Kamera Utama (Deteksi Bola/Kotak)
CAM_SURFACE_INDEX = 1    # Kamera Atas (Foto Mangrove)
CAM_UNDERWATER_INDEX = 2 # Kamera Bawah (Foto Ikan)

# --- MODEL ---
MODEL_PATH = "models/best.onnx"
INPUT_SIZE = 320  # Ukuran training model Anda

# --- NAVIGASI (TUNING PID SEDERHANA) ---
CRUISE_SPEED = 1.0        # Kecepatan maju (m/s)
MAX_TURN_SPEED = 0.8      # Kecepatan belok maks (m/s)
P_GAIN_STEERING = 0.005   # Sensitivitas belok (Makin besar, makin agresif)
SAFE_DISTANCE_PIXELS = 50 # Jarak aman toleransi tengah

# --- WAYPOINT UNDERWATER (Blue Box) ---
# Koordinat relatif (meter) dari titik start untuk misi bawah air
# (Karena kita skip deteksi visual bawah air)
UNDERWATER_WP_NORTH = 15.0 
UNDERWATER_WP_EAST = 5.0

# --- CLOUD & FIREBASE ---
CLOUDINARY_CONFIG = {
    'cloud_name': "DEB3CKBZ9",
    'api_key': "886281278537257",
    'api_secret': "F5GJ-1VDNHLPOSE..." # ISI LENGKAP
}
FIREBASE_KEY_PATH = "firebase-key.json"
FIREBASE_DB_URL = "https://test-asv-monitoring-unnes-default-rtdb.asia-southeast1.firebasedatabase.app/"
FIREBASE_NODE = "/kapal/tim-asv-01"