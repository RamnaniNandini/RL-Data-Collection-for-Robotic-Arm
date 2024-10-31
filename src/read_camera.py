import cv2

# define a video capture object
capture = cv2.VideoCapture(2)

# W = 640
# H = 480
# capture.set(cv2.CAP_PROP_FRAME_WIDTH, W)
# capture.set(cv2.CAP_PROP_FRAME_HEIGHT, H)

# img_count = 1

# while True:
#     # Capture the video frame
#     ret, frame = capture.read()

#     # Display the resulting frame
#     cv2.imshow('frame', frame)

#     key = cv2.waitKey(1)

#     if key & 0xFF == 32: # Check if the spacebar is pressed (ASCII code 32)
#         # Save the image when the spacebar is pressed
#         image_filename = f"calibration_pics_logitech/captured_image_{img_count}.jpg"
#         cv2.imwrite(image_filename, frame)
#         print(f"Image saved as {image_filename}")
#         img_count = img_count+1
#     elif key & 0xFF == 27: # the 'esc' button is set as the quitting button
#         break

# # After the loop release the cap object
capture.release()
# # Destroy all the windows
# cv2.destroyAllWindows()