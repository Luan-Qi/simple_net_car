import socket

def send_messages():
    server_ip = "192.168.199.10"
    server_port = 12345
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((server_ip, server_port))
            print(f"Connected to {server_ip}:{server_port}")
            print("Enter messages to send (type 'exit' to quit):")
            
            while True:
                message = input("> ")
                if message.lower() == 'exit':
                    break
                s.sendall(message.encode('utf-8'))
                print(f"Sent: {message}")
                
        except Exception as e:
            print(f"Connection error: {str(e)}")

if __name__ == "__main__":
    send_messages()

