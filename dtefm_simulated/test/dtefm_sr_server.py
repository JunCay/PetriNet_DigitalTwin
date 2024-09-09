import socket
import threading
import time

# 处理客户端连接
def handle_client(client_socket):
    with client_socket:
        while True:
            request = client_socket.recv(1024).decode()
            if not request:
                break
            print(f"Received: {request}")
            
            # 发送过程响应
            process_response = "$,Process response"
            client_socket.sendall(process_response.encode())
            print(f"Sent: {process_response}")

            # 模拟处理时间
            time.sleep(2)  # 可调整模拟时间

            # 发送结束响应
            end_response = "!,End response"
            client_socket.sendall(end_response.encode())
            print(f"Sent: {end_response}")

# 服务器主函数
def main():
    host = '0.0.0.0'
    port = 10110

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((host, port))
    server.listen(5)
    print(f"Server listening on {host}:{port}")

    while True:
        client_socket, addr = server.accept()
        print(f"Accepted connection from {addr}")
        client_handler = threading.Thread(target=handle_client, args=(client_socket,))
        client_handler.start()

if __name__ == "__main__":
    main()