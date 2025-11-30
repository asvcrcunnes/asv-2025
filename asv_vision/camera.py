import cv2
import threading
import time

class CameraStream:
    def __init__(self, src=0, name="Camera"):
        """
        src: Index kamera (0, 1, 2) atau path video
        name: Nama untuk debugging
        """
        self.src = src
        self.name = name
        self.stream = cv2.VideoCapture(src)
        
        # Atur resolusi kamera (sesuaikan dengan kemampuan kamera USB Anda)
        # 640x480 biasanya cukup dan stabil
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
        self.stream.set(cv2.CAP_PROP_FPS, 30)

        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False
        self.lock = threading.Lock()

    def start(self):
        print(f"[{self.name}] Starting stream on source {self.src}...")
        t = threading.Thread(target=self.update, args=())
        t.daemon = True # Thread mati otomatis jika program utama mati
        t.start()
        return self

    def update(self):
        while True:
            if self.stopped:
                self.stream.release()
                return

            (grabbed, frame) = self.stream.read()
            
            with self.lock:
                self.grabbed = grabbed
                self.frame = frame
            
            # Tidur sebentar untuk hemat CPU jika frame rate kamera rendah
            time.sleep(0.005) 

    def read(self):
        # Kembalikan frame terbaru secara thread-safe
        with self.lock:
            return self.frame.copy() if self.grabbed else None

    def stop(self):
        self.stopped = True