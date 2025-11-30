import cv2
import numpy as np
import onnxruntime as ort

class YOLO_ONNX:
    def __init__(self, model_path, input_size=320, conf_thres=0.5, iou_thres=0.4):
        print(f"[Detector] Loading model: {model_path}...")
        
        self.input_size = input_size
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        
        # Mapping Class ID (SESUAIKAN DENGAN HASIL TRAINING ANDA)
        self.classes = {0: 'red_ball', 1: 'green_ball', 2: 'green_box'}
        
        # Load ONNX Runtime
        try:
            self.session = ort.InferenceSession(model_path)
            self.input_name = self.session.get_inputs()[0].name
            self.output_names = [o.name for o in self.session.get_outputs()]
            print("[Detector] Model loaded successfully.")
        except Exception as e:
            print(f"[Detector] ERROR loading model: {e}")
            self.session = None

    def preprocess(self, image):
        """Ubah ukuran gambar dan normalisasi"""
        img_h, img_w = image.shape[:2]
        
        # Resize dengan aspect ratio (Letterbox sederhana)
        scale = min(self.input_size / img_w, self.input_size / img_h)
        new_w, new_h = int(img_w * scale), int(img_h * scale)
        resized_img = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
        
        # Canvas hitam
        padded_img = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)
        
        # Tempel gambar di tengah/pojok (di sini kita taruh pojok kiri atas untuk simplifikasi)
        dw, dh = (self.input_size - new_w) // 2, (self.input_size - new_h) // 2
        padded_img[dh:dh + new_h, dw:dw + new_w, :] = resized_img
        
        # Normalize 0-255 -> 0.0-1.0 & HWC -> CHW
        input_data = padded_img.astype(np.float32) / 255.0
        input_data = input_data.transpose(2, 0, 1) # HWC to CHW
        input_data = np.expand_dims(input_data, axis=0) # Add batch dimension
        
        return input_data, scale, dw, dh

    def detect(self, frame):
        if self.session is None: return []

        img_h, img_w = frame.shape[:2]
        
        # 1. Preprocess
        input_tensor, scale, dw, dh = self.preprocess(frame)
        
        # 2. Inference
        outputs = self.session.run(self.output_names, {self.input_name: input_tensor})
        
        # 3. Postprocess
        # Output YOLOv11/v8/v5 biasanya [batch, anchors, data]
        # Data = [x, y, w, h, class_conf_1, class_conf_2, ...]
        
        detections = outputs[0][0]
        
        boxes = []
        scores = []
        class_ids = []

        # Loop through detections
        # (Catatan: Ini asumsi format output YOLOv8/11 export default.
        # Jika outputnya [batch, 84, 8400], kita perlu transpose.
        # Kode di bawah menangani format umum [rows, 85] atau transposed)
        
        if detections.shape[0] < detections.shape[1]: 
             detections = detections.T # Transpose jika perlu (umum di v8/v11)

        for det in detections:
            # Ambil max confidence score dari class
            class_scores = det[4:] # Asumsi 4 pertama adalah bbox
            class_id = np.argmax(class_scores)
            confidence = class_scores[class_id]

            if confidence > self.conf_thres:
                cx, cy, w, h = det[:4]
                
                # Un-pad & Un-scale
                x = (cx - dw) / scale
                y = (cy - dh) / scale
                w = w / scale
                h = h / scale
                
                # Koordinat xywh -> xyxy (top-left)
                x1 = int(x - w/2)
                y1 = int(y - h/2)
                
                boxes.append([x1, y1, int(w), int(h)])
                scores.append(float(confidence))
                class_ids.append(class_id)

        # NMS (Non-Maximum Suppression)
        indices = cv2.dnn.NMSBoxes(boxes, scores, self.conf_thres, self.iou_thres)
        
        results = []
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                # Hitung titik tengah objek untuk navigasi
                center_x = int(x + w/2)
                center_y = int(y + h/2)
                
                results.append({
                    "class_id": class_ids[i],
                    "label": self.classes.get(class_ids[i], "unknown"),
                    "score": scores[i],
                    "box": [x, y, x+w, y+h], # xyxy
                    "center": [center_x, center_y]
                })
        
        return results