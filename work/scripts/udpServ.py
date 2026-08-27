#!/usr/bin/python
import socket
import sys
import subprocess

# Create a UDP socket
# socket.AF_INET specifies the address family (IPv4)
# socket.SOCK_DGRAM specifies the socket type (UDP datagrams)
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to a specific address and port
# Using '' as the IP address means it will listen on all available interfaces
# It's recommended to use a port number above 1024
host = ''
port = 32099

if 2 == len(sys.argv):
    port = int( sys.argv[1] )

server_socket.bind((host, port))

print(f"UDP server up and listening on {host}:{port}")

# The server runs indefinitely to process incoming messages
while True:
    try:
        # Receive data and the client's address from an incoming datagram
        # 1024 is the buffer size, which should be sufficient for most messages
        data, client_address = server_socket.recvfrom(1024)

        # Decode the data to a string for printing (assuming utf-8 encoding)
        message = data.decode('utf-8')
        print(f"Received message from {client_address}: {message!r}")

        # Identify command
        if "dx" in message:
            cmd = "dx"
            param = message[2:]
        elif "dy" in message:
            cmd = "dy"
            param = message[2:]
        else:
            cmd = message
            param = ""

        #print(f"rgbir_ctl.sh {cmd} {param}...")
        subprocess.run( ["sh", "/work/scripts/rgbir_ctl.sh", cmd, param] )

        # Process the message (e.g., capitalize it) and send a response back to the client
        response_message = f"{message.upper()}"
        server_socket.sendto(response_message.encode('utf-8'), client_address)
        print(f"Sent response to {client_address}: {response_message!r}")

    except KeyboardInterrupt:
        print("Server is shutting down.")
        break
    except Exception as e:
        print(f"An error occurred: {e}")

server_socket.close()

