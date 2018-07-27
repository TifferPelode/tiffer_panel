#coding= utf-8
from flask import Flask, request
import socket
import sys
from threading import Thread
from threading import Lock
import signal
import os

app = Flask(__name__)

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)



# def getClient():
class SocketServer:
    # clients = {}
    
    def __init__(self, host="0.0.0.0", port=8888):
        self.serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serv.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.serv.bind((host, port))
        self.mutex = Lock()
        self.clients = {}

    def listen(self, maxlisten=5):
        self.serv.listen(maxlisten)

        while True:
            conn, addr = self.serv.accept()
            self.addClient(conn, addr)
            print("accept a connect from %s:%d" % (addr[0], addr[1]))
    
    def run(self):
        t = Thread(target=self.listen, args=(5,))
        t.start()

    def addClient(self, conn, addr):
        self.mutex.acquire()
        self.clients[addr] = conn
        self.mutex.release()
        
    def removeClient(self, conn, addr):
        self.mutex.acquire()
        del self.clients[addr]
        self.mutex.release()

    def sendToAll(self, text):
        self.mutex.acquire()
        print("send count %s" % len(self.clients.keys()))
        for addr, conn in self.clients.items():
            try:
                n = conn.send(text)
            except Exception as e:
                conn.close()
                del self.clients[addr]

        self.mutex.release()

server = SocketServer()


@app.route('/', methods=['GET'])
def hello_world():
    global server
    # num = int(request.args['num'])
    # if(num == 1):
    #     print(1)
    # elif(num == 2):
    #     print(2)
    # elif(num == 3):
    #     print(3)
    msg='welcome!' + "\r\n"
    try:
        server.sendToAll(msg.encode('utf-8'))
    except Exception as e:
        print(e)
    return 'Hello World!'


def quit(signum, frame):
    print 'You choose to stop me.'
    os.kill(os.getpid(),signal.SIGKILL)

if __name__ == '__main__':
    signal.signal(signal.SIGINT,quit)
    # #serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # host = '0.0.0.0'
    # port = 8765
    # serversocket.bind((host, port))
    # serversocket.listen(5)
    server.run()
    app.run(host='0.0.0.0', port='8765')
