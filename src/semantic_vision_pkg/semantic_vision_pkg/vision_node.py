import cv2
from ultralytics import YOLO

def run_vision_node():
    # 1. Load the "Brain"
    # We use 'yolov8n-seg.pt' (Nano model) for speed on your laptop/Edge device.
    # It will auto-download the weight file (approx 6MB) on the first run.
    print("Loading YOLOv8 Segmentation model...")
    model = YOLO('yolov8n-seg.pt')

    # 2. Open the "Eye" (Webcam)
    # 0 is usually the default webcam. If you have an external one, try 1.
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    # ... previous imports and setup ...

    print("Vision System Active. Point at a Person vs. a Bottle.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Run inference
        results = model.predict(source=frame, save=False, conf=0.5, verbose=False)
        result = results[0] # We only have one frame
        
        # Visualize (Human View)
        annotated_frame = result.plot()
        
        # --- THE DRONE BRAIN (Logic View) ---
        status_message = "Path Clear - Flying"
        color = (0, 255, 0) # Green

        # Check what objects are detected
        if result.boxes:
            # Get all detected class IDs in the frame
            # .cpu().numpy() converts the Tensor to a standard Python list
            classes = result.boxes.cls.cpu().numpy() 
            
            # COCO Class IDs: 0 = Person, 39 = Bottle, 56 = Chair
            if 0 in classes:
                status_message = "CRITICAL: PERSON DETECTED! (Cost=255)"
                color = (0, 0, 255) # Red
            elif 39 in classes or 56 in classes:
                status_message = "OBSTACLE: Object Detected (Cost=128)"
                color = (0, 165, 255) # Orange

        # Simulate the Decision on screen
        cv2.putText(annotated_frame, status_message, (50, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
        
        # ------------------------------------

        cv2.imshow('Semantic-Aware Drone Vision', annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    run_vision_node()