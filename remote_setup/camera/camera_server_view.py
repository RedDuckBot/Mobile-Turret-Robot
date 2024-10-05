import cv2, socket, pickle
import numpy as np 

"""Program acts as a UDP server for viewing Stroam's video camera."""

remote_host_ip = "TODO_CHANGE"
port = 6669

video_server_UDPsocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
video_server_socket.bind((remote_host_ip,port))

print("Video Server ready for Stroam's view")
while True:
    payload = video_server_UDPsocket.recvfrom(1000000)
    stroam_ip = payload[1][0]
    data = payload[0]

    data = pickle.loads(data)

    img = cv2.imdecode(data, cv2.IMREAD_COLOR)
    cv2.imshow("Im Server", img)

    if cv2.waitKey(5) & 0xFF == 113:
        break

cv2.destroyAllWindows()
video_server_UDPsocket.close()
