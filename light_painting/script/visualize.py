import cv2

# Load the video
video_capture = cv2.VideoCapture('/path/to/file')

# Check if the video capture is open
if not video_capture.isOpened():
    print("Error: Unable to open video file.")
    exit()

# Get video properties
frame_width = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(video_capture.get(cv2.CAP_PROP_FPS))

# Define the codec and create a VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use MP4V codec
output_video = cv2.VideoWriter('output_video.mp4', fourcc, fps, (frame_width, frame_height))

# List to store centroid positions and corresponding colors across frames
centroid_info = []

while True:
    ret, frame = video_capture.read()
    if not ret:
        print("End of video.")
        break

    # Apply background subtraction (you may need to experiment with different methods)
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, threshold_frame = cv2.threshold(gray_frame, 50, 255, cv2.THRESH_BINARY)

    # Find contours
    contours, _ = cv2.findContours(threshold_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Iterate through contours
    for contour in contours:
        # Filter contours based on area or other criteria if needed
        area = cv2.contourArea(contour)
        if area > 10:
            # Get centroid of the contour
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Determine color of the contour
                color = frame[cy, cx]  # Get the color of the centroid pixel
                if color[2] > color[1] and color[2] > color[0]:  # Red channel is dominant
                    circle_color = (66, 66, 252)  # Red
                elif color[1] > color[0] and color[1] > color[2]:  # Green channel is dominant
                    circle_color = (151, 255, 120)  # Green
                else:
                    circle_color = (255, 227, 92)  # Blue

                # Store centroid position and corresponding color in the list
                centroid_info.append((cx, cy, circle_color))

    # Draw circles on the frame for each centroid position with corresponding colors
    for cx, cy, circle_color in centroid_info:
        cv2.circle(frame, (cx, cy), 3, circle_color, -1)

    # Write the frame to the output video
    output_video.write(frame)

    # Display the frame
    cv2.imshow('Video', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release video capture and close windows
video_capture.release()
output_video.release()
cv2.destroyAllWindows()
