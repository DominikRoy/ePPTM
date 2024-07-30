import socket
import ssl

# buffer size
BUFF_SIZE = 4096
# debug mode
DEBUG_MODE = False
# seqeunce indicating end of message
END_OF_MSG = "a9b78a88415522de2e2e625ce8a88ac8"

# buffer for next message
next_message_buffer = ""


def create_server_socket(HOST, PORT, private_key_file_path, certificate_file_path):
    if DEBUG_MODE:
        print(f"\u001b[33mCreating server socket on {HOST}:{PORT} ...\u001b[0m")
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server = ssl.wrap_socket(
        server_socket,
        server_side=True,
        keyfile="./certificates/" + private_key_file_path,
        certfile="./certificates/" + certificate_file_path
    )
    server.bind((HOST, PORT))
    server.listen(0)
    if DEBUG_MODE:
        print(f"\u001b[33mServer socket created on {HOST}:{PORT}\u001b[0m")
    return server


def create_client_socket(HOST, PORT, private_key_file_path, certificate_file_path):
    if DEBUG_MODE:
        print(f"\u001b[33mConnecting to {HOST}:{PORT} ...\u001b[0m")
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    client = ssl.wrap_socket(
        client_socket,
        keyfile="./certificates/" + private_key_file_path,
        certfile="./certificates/" + certificate_file_path
    )
    client.connect((HOST, PORT))
    if DEBUG_MODE:
        print(f"\u001b[33mCreated client socket connected to {HOST}:{PORT}\u001b[0m")
    return client


def receive(connection):
    global next_message_buffer
    msg = next_message_buffer

    while True:
        data = connection.recv(BUFF_SIZE)
        msg = msg + data.decode('utf-8')
        if END_OF_MSG in msg:
            break

    if msg.endswith(END_OF_MSG):
        msg = msg.replace("}" + END_OF_MSG, "}")
        next_message_buffer = ""
    else:
        index_end = msg.find("}" + END_OF_MSG)
        next_message_buffer = msg[index_end+1+len(END_OF_MSG):len(msg)+1]
        msg = msg[0:index_end + 1]

    if DEBUG_MODE:
        print(f"\033[1;32mReceived: {msg}\u001b[0m")
    return msg


def send(connection, msg):
    if DEBUG_MODE:
        print(f"\u001b[34mSend: {msg}\u001b[0m")

    msg = msg + END_OF_MSG
    connection.send(msg.encode("utf-8"))


def wrong_code_received(expected, got):
    raise Exception(f"\u001b[31mExpected code '{expected}', but received code '{got}'\u001b[0m")
