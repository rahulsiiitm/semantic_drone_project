import cv2
import numpy as np
from ultralytics import YOLO

def run_vision_node():
    print("Loading YOLOv8-Seg (Demo Mode)...")
    model = YOLO('yolov8n-seg.pt')
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Webcam not found.")
        return

    # Define the "Drone's" imaginary position (Bottom center of screen)
    drone_x = 320  # Middle of a 640px wide screen
    drone_y = 480  # Bottom

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Resize for consistent math (640x480)
        frame = cv2.resize(frame, (1280, 960))
        
        # Make a copy to draw "Augmented Reality" graphics
        display_frame = frame.copy()
        
        # Run AI
        results = model.predict(source=frame, save=False, conf=0.5, verbose=False)
        result = results[0]

        obstacle_center = None
        
        if result.boxes:
            classes = result.boxes.cls.cpu().numpy()
            boxes = result.boxes.xyxy.cpu().numpy() # Bounding boxes

            for i, cls_id in enumerate(classes):
                x1, y1, x2, y2 = map(int, boxes[i])
                cx, cy = int((x1+x2)/2), int((y1+y2)/2)

                # RULE: PERSON (ID 0) = LETHAL OBSTACLE
                if cls_id == 0:
                    # 1. Draw the Object (Red Box)
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(display_frame, "HUMAN (Cost: 255)", (x1, y1-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    # 2. Draw the "Safety Bubble" (Yellow Circle)
                    # We simulate a 2-meter radius (e.g., 100 pixels)
                    radius = 120 
                    # Draw a transparent overlay
                    overlay = display_frame.copy()
                    cv2.circle(overlay, (cx, cy), radius, (0, 255, 255), -1) # Filled Yellow
                    cv2.addWeighted(overlay, 0.3, display_frame, 0.7, 0, display_frame)
                    
                    # Save center for path planning visual
                    obstacle_center = (cx, cy)

        # 3. Visualize "Path Planning" (The Green Line)
        # If there is an obstacle, draw a curved path around it
        start_point = (320, 480) # Bottom Center
        target_point = (320, 100) # Top Center (Goal)

        if obstacle_center:
            # Simple logic: If obstacle is in center, path goes Left or Right
            obs_x, obs_y = obstacle_center
            
            if obs_x < 320: # Obstacle is on Left
                # Curve Right
                control_point = (500, 300) 
            else: # Obstacle is on Right
                # Curve Left
                control_point = (140, 300)
            
            # Draw a Bezier-like curve (Quadratic)
            # (Simplified as connected lines for OpenCV)
            cv2.line(display_frame, start_point, control_point, (0, 255, 0), 3)
            cv2.line(display_frame, control_point, target_point, (0, 255, 0), 3)
            cv2.circle(display_frame, control_point, 5, (0, 255, 0), -1)
            cv2.putText(display_frame, "RE-PLANNING PATH...", (control_point[0]-50, control_point[1]-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            # Path is clear -> Straight Line
            cv2.line(display_frame, start_point, target_point, (0, 255, 0), 3)
            cv2.putText(display_frame, "PATH CLEAR", (280, 400),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow('Semantic-Aware Navigation Logic', display_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    run_vision_node()