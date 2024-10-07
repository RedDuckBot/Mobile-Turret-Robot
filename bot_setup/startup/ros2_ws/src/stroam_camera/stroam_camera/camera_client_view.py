import cv2, socket, pickle, os
import numpy as np

video_client_UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video_client_UDP_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF,1000000)

remote_server_ip = "TODO_CHANGE" 
server_port = 6669 

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

print("Stroam's view is ready for live video stream")
while cap.isOpened():
    ret, img = cap.read()

    #cv2.imshow("Img client", img)
    
    ret, buffer = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 30])

    x_as_bytes = pickle.dumps(buffer)     

    video_client_UDP_socket.sendto((x_as_bytes),(remote_server_ip,server_port))

    if cv2.waitKey(5) & 0xFF == 113:
        break 

cv2.destroyAllWindows()
cap.release()
