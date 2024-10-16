import socket
import csv
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

# 設定伺服器的IP位址和埠號
HOST = '0.0.0.0'  # 本地回路
PORT = 65432      # 任意非保留埠號

# 儲存接收到的資料
x_data, y_data, z_data = [], [], []

# 創建一個csv檔案，並寫入標頭
with open('coordinates.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['X', 'Y', 'Z'])  # 寫入CSV檔案的標頭

# 創建socket對象
def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))  # 綁定伺服器的IP位址和埠號
        s.listen()            # 開始監聽進來的連線
        print(f"Server is listening on {HOST}:{PORT}...")

        conn, addr = s.accept()  # 接受新的連線
        with conn:
            print(f"Connected by {addr}")
            conn.sendall(b"10001")
            while True:
                data = conn.recv(1024)  # 接收資料
                if not data:
                    break
                received_data = data.decode('utf-8').strip()
                print(f"Received: {received_data}")
                
                # 解析接收到的資料
                try:
                    # 假設資料格式為 X: -3, Y: -9, Z: 1053
                    parts = received_data.split(',')
                    x = int(parts[0].split(':')[1].strip().split()[0])  # 去掉多餘字元
                    y = int(parts[1].split(':')[1].strip().split()[0])  # 去掉多餘字元
                    z = int(parts[2].split(':')[1].strip().split()[0])  # 去掉多餘字元

                    # 將X, Y, Z寫入csv檔案
                    with open('coordinates.csv', mode='a', newline='') as file:
                        writer = csv.writer(file)
                        writer.writerow([x, y, z])

                    # 更新資料列表
                    x_data.append(x)
                    y_data.append(y)
                    z_data.append(z)

                except Exception as e:
                    print(f"Error parsing data: {e}")

                conn.sendall(data)  # 回傳資料給客戶端

# 實時繪圖函數
def animate(i):
    if len(x_data) > 0:
        plt.cla()
        plt.plot(x_data[-50:], label='X')
        plt.plot(y_data[-50:], label='Y')
        plt.plot(z_data[-50:], label='Z')
        plt.xlabel('Time (samples)')
        plt.ylabel('Value')
        plt.title('Real-Time Sensor Data')
        plt.legend(loc='upper right')

# 啟動即時圖表
fig = plt.figure()
ani = FuncAnimation(fig, animate, interval=100)  # 每100毫秒更新一次

# 使用多線程來同時運行伺服器和顯示圖表
server_thread = threading.Thread(target=start_server)
server_thread.start()

# 顯示圖表
plt.show()

# 等待伺服器線程結束
server_thread.join()
