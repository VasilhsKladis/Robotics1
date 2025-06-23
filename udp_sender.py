import asyncio
import socket
import struct
from typing import Optional

# === CONFIGURATION ===
ESP32_IP = '192.168.137.137'
ESP32_PORT = 4210
PC_UDP_PORT = 4211

# === UDP SETUP ===
class UDPSystem:
    def __init__(self):
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv.bind(('0.0.0.0', PC_UDP_PORT))
        self.sock_recv.settimeout(0.1)
        
        # SLAM packet format (matches ESP32's struct)
        self.slam_format = 'Iffffffhh'  # uint32_t + 6x float + 2x int16
        self.slam_size = struct.calcsize(self.slam_format)
        
        self.running = True

    async def receive_task(self):
        """Async task for receiving and decoding SLAM data"""
        print(f"[SLAM] Listening on port {PC_UDP_PORT}")
        while self.running:
            try:
                data, addr = self.sock_recv.recvfrom(1024)
                if len(data) == self.slam_size:
                    self.decode_slam_packet(data)
            except socket.timeout:
                await asyncio.sleep(0.01)
            except Exception as e:
                print(f"[SLAM Error] {e}")
                break

    def decode_slam_packet(self, data: bytes):
        """Decode binary SLAM packet"""
        try:
            (timestamp, gx, gy, gz, ax, ay, az, speed, angle) = \
                struct.unpack(self.slam_format, data)
            
           
        except struct.error:
            print(f"[SLAM] Invalid packet: {data.hex()}")

    async def send_command(self, speed: int, angle: int):
        """Send motor commands to ESP32"""
        try:
             # Ensure proper range checking
            speed = max(-1000, min(1000, int(speed)))
            angle = max(-360, min(360, int(angle)))
            payload = struct.pack('ii', speed, angle)
            self.sock_send.sendto(payload, (ESP32_IP, ESP32_PORT))
            print(f"[CMD] Sent: speed={speed}, angle={angle}")
        except Exception as e:
            print(f"[CMD Error] {e}")

    async def input_task(self):
        """Async task for handling user input"""
        print("== UDP Control Client ==")
        print("Format: <speed> <angle> (e.g., '150 90')")
        print("Type 'quit' to exit\n")
        
        while self.running:
            try:
                # Non-blocking input using asyncio
                cmd = await asyncio.get_event_loop().run_in_executor(
                    None, 
                    input, 
                    "Enter command (speed angle): "
                )
                
                if cmd.strip().lower() == 'quit':
                    self.running = False
                    break
                    
                try:
                    speed, angle = map(int, cmd.split())
                    await self.send_command(speed, angle)
                except ValueError:
                    print("Invalid input! Use: <speed> <angle>")
                    
            except (KeyboardInterrupt, EOFError):
                self.running = False
                break

    async def cleanup(self):
        """Properly close sockets"""
        self.sock_send.close()
        self.sock_recv.close()
        print("\n[System] Closed all connections")

async def main():
    system = UDPSystem()
    
    # Send initialization packet
    system.sock_send.sendto(b'INIT', (ESP32_IP, ESP32_PORT))
    
    # Run concurrent tasks
    try:
        await asyncio.gather(
            system.receive_task(),
            system.input_task()
        )
    finally:
        await system.cleanup()

if __name__ == '__main__':
    asyncio.run(main())