import time
import zmq
import serial


def connect():
    global port
    while True:
        try:
            port = serial.Serial("/dev/serial/by-id/usb-JeVois_Inc_JeVois-A33_Smart_Camera-if02", baudrate=115200, timeout=3.0)
            break
        except:
            socket.send_string("PNP unable to contact JeVois")
            print("unable to contact JeVois")
            time.sleep(0.1)
    

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5800")

connect()


while True:
    try:
        rcv = port.readline()
        print(rcv)
        socket.send_string("PNP " + rcv.decode('utf-8'))
    except:
        socket.send_string("PNP lost connection! attempting to recover")
        connect()

