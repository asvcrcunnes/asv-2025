import cv2
import time
import numpy as np
import config
from asv_vision.camera import CameraStream
from asv_vision.detector import YOLO_ONNX
from asv_comms.mavlink_interface import MavlinkInterface
from asv_monitoring.web_publisher import WebPublisher

# --- FUNGSI BANTU ---
def center_crop(frame, size):
    """Memotong bagian tengah frame menjadi ukuran size x size"""
    h, w, _ = frame.shape
    cx, cy = w // 2, h // 2
    x1 = cx - size // 2
    y1 = cy - size // 2
    # Pastikan tidak keluar batas
    x1 = max(0, x1)
    y1 = max(0, y1)
    return frame[y1:y1+size, x1:x1+size]

def calculate_steering(detections, frame_center_x):
    """
    Logika Navigasi Sederhana (Visual Servoing).
    Mengembalikan kecepatan lateral (vy) berdasarkan posisi bola.
    """
    green_ball = None
    red_ball = None
    
    # Cari bola dengan confidence tertinggi
    for det in detections:
        if det['label'] == 'green_ball':
            if green_ball is None or det['score'] > green_ball['score']:
                green_ball = det
        elif det['label'] == 'red_ball':
            if red_ball is None or det['score'] > red_ball['score']:
                red_ball = det

    target_x = frame_center_x # Default: Lurus (tengah frame)

    if green_ball and red_ball:
        # Ada dua bola, target adalah tengah-tengahnya
        target_x = (green_ball['center'][0] + red_ball['center'][0]) / 2
    elif green_ball:
        # Hanya hijau (kanan), kita harus berada di kirinya
        # Offset misal 100px ke kiri dari bola hijau
        target_x = green_ball['center'][0] - 120 
    elif red_ball:
        # Hanya merah (kiri), kita harus berada di kanannya
        # Offset misal 100px ke kanan dari bola merah
        target_x = red_ball['center'][0] + 120
    else:
        # Tidak ada bola, jalan lurus (atau cari bola)
        return 0.0, False # False artinya "Lost Visual"

    # Hitung Error (Seberapa jauh target dari tengah frame)
    error_x = target_x - frame_center_x
    
    # P-Controller untuk Steering
    # Jika error positif (target di kanan), vy positif (geser kanan)
    vy = error_x * config.P_GAIN_STEERING
    
    # Batasi kecepatan belok
    vy = max(min(vy, config.MAX_TURN_SPEED), -config.MAX_TURN_SPEED)
    
    return vy, True # True artinya "Visual Lock"

# --- MAIN PROGRAM ---
def main():
    # 1. Inisialisasi Modul
    mav = MavlinkInterface(config.MAVLINK_CONNECTION, config.MAVLINK_BAUD)
    web = WebPublisher(config.FIREBASE_KEY_PATH, config.FIREBASE_DB_URL, config.FIREBASE_NODE, config.CLOUDINARY_CONFIG)
    cam = CameraStream(config.CAMERA_INDEX).start()
    detector = YOLO_ONNX(config.MODEL_PATH, input_size=config.INPUT_SIZE)

    # Start Threads
    mav.start()
    web.start()

    # Tunggu kamera panas
    time.sleep(2.0)
    print("[Main] System Ready. Waiting for ARM...")

    # --- STATE MACHINE VARIABLES ---
    state = "WAIT_FOR_ARM"
    balls_passed = 0
    mission_start_time = 0
    
    try:
        while True:
            # 1. Ambil Data
            frame = cam.read()
            telemetry = mav.get_state()
            
            if frame is None: continue

            # Update Web
            web.update_telemetry(telemetry)

            # Pre-processing untuk YOLO (Crop 320x320)
            frame_cropped = center_crop(frame, config.INPUT_SIZE)
            
            # Deteksi
            detections = detector.detect(frame_cropped)
            
            # Visualisasi (Optional - tampilkan di layar untuk debug)
            # (Anda bisa menambahkan cv2.rectangle di sini menggunakan data detections)

            # --- FINITE STATE MACHINE (FSM) ---
            
            if state == "WAIT_FOR_ARM":
                if telemetry['armed']:
                    print("[FSM] Vehicle ARMED. Starting Mission!")
                    state = "NAV_BALLS"
                    web.update_mission_status("start", "Done")
                    mav.set_mode("GUIDED")
                    mission_start_time = time.time()

            elif state == "NAV_BALLS":
                # Hitung kemudi
                vy, has_visual = calculate_steering(detections, config.INPUT_SIZE // 2)
                vx = config.CRUISE_SPEED
                
                # Kirim Perintah ke ArduRover
                mav.send_guided_velocity(vx, vy, 0)
                
                # Cek Green Box (Transisi ke Surface Imaging)
                # Jika Green Box terdeteksi dengan confidence tinggi dan dekat (kotak besar)
                for det in detections:
                    if det['label'] == 'green_box':
                        # Logika sederhana: jika lebar box > 50px, anggap sudah dekat
                        box_width = det['box'][2] - det['box'][0] 
                        if box_width > 50:
                            print("[FSM] Green Box Detected! Switching to CAPTURE.")
                            state = "CAPTURE_SURFACE"
                            mav.send_guided_velocity(0, 0, 0) # STOP

            elif state == "CAPTURE_SURFACE":
                # 1. Stop Kapal (Loiter/Hold)
                mav.set_mode("HOLD")
                time.sleep(1) # Tunggu stabil
                
                # 2. Ambil Foto & Upload
                web.update_mission_status("surface_imaging", "In Progress")
                # Kita pakai frame asli (full resolution) untuk upload, bukan yg di-crop
                web.upload_image(frame, "surface", telemetry)
                time.sleep(2) # Simulasi waktu upload/proses
                
                print("[FSM] Surface Mission Done. Going Underwater.")
                state = "GOTO_UNDERWATER"
                mav.set_mode("GUIDED")

            elif state == "GOTO_UNDERWATER":
                # Navigasi Buta ke Waypoint (Logic Sederhana)
                # Hitung jarak ke titik target
                dist_north = config.UNDERWATER_WP_NORTH - telemetry['local_x']
                dist_east = config.UNDERWATER_WP_EAST - telemetry['local_y']
                distance = np.sqrt(dist_north**2 + dist_east**2)
                
                if distance < 1.0: # Sampai (toleransi 1 meter)
                    print("[FSM] Arrived at Underwater WP.")
                    state = "CAPTURE_UNDERWATER"
                    mav.send_guided_velocity(0, 0, 0)
                else:
                    # Kirim velocity vector ke arah target
                    # (Normalisasi vektor lalu kalikan speed)
                    vx = (dist_north / distance) * config.CRUISE_SPEED
                    vy = (dist_east / distance) * config.CRUISE_SPEED
                    mav.send_guided_velocity(vx, vy, 0)

            elif state == "CAPTURE_UNDERWATER":
                mav.set_mode("HOLD")
                time.sleep(1)
                
                web.update_mission_status("underwater_imaging", "In Progress")
                web.upload_image(frame, "underwater", telemetry) # Asumsi kamera sama
                time.sleep(2)
                
                print("[FSM] All Missions Done. RTL.")
                state = "RTL"
                mav.set_mode("RTL")
                web.update_mission_status("finish", "In Progress")

            elif state == "RTL":
                # Monitor jarak ke Home (0,0)
                dist_home = np.sqrt(telemetry['local_x']**2 + telemetry['local_y']**2)
                if dist_home < 1.0:
                    print("[FSM] Home Reached. Disarming.")
                    # mav.disarm() # Optional: auto disarm
                    web.update_mission_status("finish", "Done")
                    break # Keluar loop atau standby

            # --- END FSM ---
            
            # Tampilkan Debug View
            cv2.imshow("ASV Vision", frame_cropped)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("[Main] Stopping...")
    finally:
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()