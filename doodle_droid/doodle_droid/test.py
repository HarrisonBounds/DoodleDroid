import cv2
import numpy as np

class BackgroundRemover:
    def __init__(self):
        """
        Initialize background subtraction and segmentation components
        """
        # Create background subtractor
        self.back_sub = cv2.createBackgroundSubtractorMOG2(
            history=100,  # Number of frames used to create background model
            varThreshold=50,  # Threshold for detection of motion
            detectShadows=True  # Detect shadows
        )
        
    def remove_background(self, frame):
        """
        Remove background and create a segmented image
        
        Args:
        frame (numpy.ndarray): Input frame from webcam
        
        Returns:
        numpy.ndarray: Segmented frame with black background
        """
        # Apply background subtraction
        fg_mask = self.back_sub.apply(frame)
        
        # Threshold the mask to create a binary image
        _, thresh = cv2.threshold(fg_mask, 244, 255, cv2.THRESH_BINARY)
        
        # Perform morphological operations to reduce noise
        kernel = np.ones((3,3), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        
        # Create a black background image
        black_background = np.zeros_like(frame)
        
        # Copy the original frame to black background using the mask
        black_background[thresh == 255] = frame[thresh == 255]
        
        return black_background, thresh

def detect_face_contours(frame, face_cascade):
    """
    Detect face and extract its contours.
    
    Args:
    frame (numpy.ndarray): Input image frame
    face_cascade (cv2.CascadeClassifier): Trained face detector
    
    Returns:
    tuple: Processed frame with face contours, list of face contours
    """
    # Convert frame to grayscale for face detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect faces
    faces = face_cascade.detectMultiScale(
        gray, 
        scaleFactor=1.1,  
        minNeighbors=5,   
        minSize=(30, 30)  
    )
    
    # Create a copy of the original frame to draw on
    frame_with_contours = frame.copy()
    face_contours = []
    
    # Process each detected face
    for (x, y, w, h) in faces:
        # Extract the face region
        face_roi = gray[y:y+h, x:x+w]
        
        # Apply some preprocessing to improve contour detection
        blurred = cv2.GaussianBlur(face_roi, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # Find contours in the face region
        contours, _ = cv2.findContours(
            thresh, 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Filter and adjust contours to original frame coordinates
        significant_contours = []
        for cnt in contours:
            # Adjust contour coordinates back to the original frame
            adjusted_cnt = cnt + [x, y]
            
            # Filter contours by area
            if cv2.contourArea(adjusted_cnt) > 50:
                significant_contours.append(adjusted_cnt)
                
        # Draw face rectangle
        cv2.rectangle(
            frame_with_contours, 
            (x, y), 
            (x+w, y+h), 
            (255, 0, 0),  # Blue color
            2  # Thickness
        )
        
        # Draw face contours
        cv2.drawContours(
            frame_with_contours, 
            significant_contours, 
            -1,  # Draw all contours
            (0, 255, 0),  # Green color
            2  # Thickness of contour lines
        )
        
        # Combine contours
        face_contours.extend(significant_contours)
    
    return frame_with_contours, face_contours

def process_webcam():
    """
    Process webcam input, detecting and displaying face contours in real-time.
    """
    # Attempt to load the Haar Cascade classifier
    face_cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
    face_cascade = cv2.CascadeClassifier(face_cascade_path)
    
    # Initialize background remover
    bg_remover = BackgroundRemover()
    
    # Open the default camera
    cap = cv2.VideoCapture(0)
    
    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    while True:
        # Read a frame from the camera
        ret, frame = cap.read()
        
        # Break the loop if no frames are captured
        if not ret:
            break
        
        # Flip the frame horizontally for a more natural mirror effect
        frame = cv2.flip(frame, 1)
        
        # Remove background
        black_bg_frame, mask = bg_remover.remove_background(frame)
        
        # Detect face contours in the frame
        frame_with_contours, face_contours = detect_face_contours(black_bg_frame, face_cascade)
        
        # Display the number of face contours detected
        cv2.putText(
            frame_with_contours, 
            f"Face Contours: {len(face_contours)}", 
            (10, 30), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            1, 
            (255, 0, 0), 
            2
        )
        
        # Show the frame with face contours and background mask
        cv2.imshow('Webcam Face Contour Detection', frame_with_contours)
        cv2.imshow('Background Mask', mask)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release resources
    cap.release()
    cv2.destroyAllWindows()

def main():
    process_webcam()

if __name__ == "__main__":
    main()