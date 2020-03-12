#Object detection

#install tensorflow, opencv-python, keras, imageAI
from imageai.Detection import ObjectDetection
import os
import cv2
import socket as sk
import json
import struct

#initialize the model
detector = ObjectDetection()

model_path = r'C:\Users\alber\Desktop\droneProject\objectDetection\models\yolo-tiny.h5'

detector.setModelTypeAsTinyYOLOv3()
detector.setModelPath(model_path)
detector.loadModel()

#start connection with the server
id = 0

address = 'localhost'
port = 9999

client_socket = sk.socket(sk.AF_INET, sk.SOCK_STREAM) 
client_socket.connect((address, port))

#send starting message
starting_message = {"id":id, "type":"register_application", "data":{"id":id}}
starting_message = json.dump(starting_message)
starting_message = starting_message.encode();
client_socket.send(starting_message)

#receive starting message
receiving_message =client_socket.recv(1024) #1024 bytes (size of the ethernet package)
receiving_message = receiving_message.decode()  #decode the received message from binary to string
receiving_message = json.loads(receiving_message)

if (receiving_message["type"] == "ok" and receiving_message["data"]["msg_id"] == id):
  id = id + 1
  #infinite loop
  while(True):
    #send image request
    img_request = {"id":id, "type":"req_camera_pic"}
    img_request = json.dump(img_request)
    img_request = img_request.encode();
    client_socket.send(img_request)

    #get the image path
    receiving_message =client_socket.recv(1024) #1024 bytes (size of the ethernet package)
    receiving_message = receiving_message.decode()  #decode the received message from binary to string
    receiving_message = json.loads(receiving_message)
    id = id + 1

    #detecttion
    detections = detector.detectObjectsFromImage(input_image=receiving_message["data"]["path"], minimum_percentage_probability=40)

    #extract the information from the detection results
    objects_id = detector.numbers_to_names

    objects_result = []

    for id, name in objects_id.items():
      for eachObject in detections:
        if name == eachObject["name"]:
          id_result = id
          probability = eachObject["percentage_probability"]
          x1 = eachObject["box_points"]["x1"]
          y1 = eachObject["box_points"]["y1"]
          x2 = eachObject["box_points"]["x2"]
          y2 = eachObject["box_points"]["y2"]
          objects_result = struct.pack('Bfffff', id_result, probability, x1, y1, x2, y2)
    
    #communication back
    anwer_message = {"id":id, "type":"message", "data":id_result}
    anwer_message = json.dump(anwer_message)
    anwer_message = anwer_message.encode();
    client_socket.send(anwer_message)

    #wait acknoledgement
    receiving_message =client_socket.recv(1024) #1024 bytes (size of the ethernet package)
    receiving_message = receiving_message.decode()  #decode the received message from binary to string
    receiving_message = json.loads(receiving_message)
    
    if(receiving_message["type"] == 'ok' and receiving_message["data"]["msg_id"] == id):
      id = id + 1
    else:
      print("Error")
else:
  print("Error")
