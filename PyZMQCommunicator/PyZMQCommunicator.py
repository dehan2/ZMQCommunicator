import zmq

class PyZMQCommunicator:
    def __init__(self):
        context = zmq.Context()
        self.server_socket = context.socket(zmq.REP)
        self.client_socket = context.socket(zmq.REQ)
    
    def bind_server(self, address):
        self.server_socket.bind(address)

    def connect_client(self, address):
        self.client_socket.connect(address)

    def receive(self):
         self.receivedMessage = self.server_socket.recv()
         return self.receivedMessage

    def reply_to_client(self, msg):
        result = self.server_socket.send(msg)
        return result

    def send(self, msg):
        result = self.client_socket.send(msg)
        return result

    def receive_reply(self):
         reply = self.client_socket.recv()
         return self.receivedMessage



