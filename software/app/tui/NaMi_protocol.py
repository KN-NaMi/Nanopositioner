import socket
import json

# --- Protocol Constants ---
DISCOVERY_PORT = 5005
BROADCAST_IP = '255.255.255.255'
DISCOVERY_MESSAGE = b"DISCOVER_NAMI_DEVICES"
DISCOVERY_TIMEOUT = 5.0
TCP_TIMEOUT = 5.0
DEFAULT_TCP_PORT = 6000

def discover_devices(device_type_filter="dac"):
    found_devices = []
    print(">>> Scanning network for NaMi devices...")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.settimeout(DISCOVERY_TIMEOUT)
            sock.sendto(DISCOVERY_MESSAGE, (BROADCAST_IP, DISCOVERY_PORT))

            while True:
                try:
                    data, addr = sock.recvfrom(1024)
                    device_info = json.loads(data.decode('utf-8'))
                    if device_info.get("type") == device_type_filter:
                        device_info['ip_address'] = addr[0]
                        found_devices.append(device_info)
                        print(f"    ...found device '{device_info.get('device_id', 'N/A')}' of type '{device_type_filter}'")
                except socket.timeout:
                    break
                except (json.JSONDecodeError, UnicodeDecodeError):
                    continue
    except Exception as e:
        print(f"[ERROR] An error occurred during network scan: {e}")

    if not found_devices:
        print(f">>> No devices of type '{device_type_filter}' were found.")
    else:
        print(f">>> Scan finished. Found {len(found_devices)} matching devices.")
    return found_devices

# Manages a single, persistent TCP session with a NaMi device
class NamiConnection:
    def __init__(self):
        self.sock = None
        self.is_connected = False
        self.session_id = None
        self.device_ip = None
        self.device_port = None

    # Establishes a TCP connection and initializes the NaMi session
    def connect(self, ip, port=DEFAULT_TCP_PORT):
        if self.is_connected:
            print("[WARNING] Already connected.")
            return True

        try:
            print(f">>> Connecting to {ip}:{port}...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(TCP_TIMEOUT)
            self.sock.connect((ip, port))
            print(">>> TCP connection successful. Initializing NaMi session...")

            # Automatically send INIT command after connection
            init_command = {"cmd": "INIT", "client": "Python TUI v1.0"}
            init_response = self.send_command(init_command, is_init=True)

            if init_response and init_response.get("status") == "OK":
                self.session_id = init_response.get("session_id")
                self.is_connected = True
                self.device_ip = ip
                self.device_port = port
                print(f">>> NaMi session initialized successfully! Session ID: {self.session_id}")
                return True
            else:
                print("[ERROR] Failed to initialize NaMi session. Closing connection.")
                self.sock.close()
                self.sock = None
                return False

        except socket.timeout:
            print(f"[ERROR] Connection to {ip} timed out.")
        except ConnectionRefusedError:
            print(f"[ERROR] Connection to {ip} was refused. Check if the device is listening.")
        except Exception as e:
            print(f"[ERROR] Unexpected error during connection: {e}")

        self.sock = None
        return False

    # Closes the active TCP connection
    def disconnect(self):
        if not self.is_connected:
            return

        print(">>> Disconnecting...")
        try:
            if self.sock:
                self.sock.close()
        except Exception as e:
            print(f"[ERROR] An error occurred while closing the socket: {e}")
        finally:
            self.sock = None
            self.is_connected = False
            self.session_id = None
            self.device_ip = None
            self.device_port = None
            print(">>> Disconnected.")


    # Sends a JSON command and returns the JSON response as a dict
    def send_command(self, command_dict, is_init=False):
        if not self.sock or (not self.is_connected and not is_init):
            print("[ERROR] No active connection. Cannot send command.")
            return None

        try:
            command_json = json.dumps(command_dict)
            self.sock.sendall(command_json.encode('utf-8'))

            response_data = self.sock.recv(2048) 
            if not response_data:
                print("[ERROR] Connection was closed by the device.")
                self.disconnect()
                return None
            
            response_dict = json.loads(response_data.decode('utf-8'))
            return response_dict

        except (socket.timeout, ConnectionResetError, BrokenPipeError) as e:
            print(f"[NETWORK ERROR] {e}. Closing connection.")
            self.disconnect()
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            print(f"[DATA ERROR] Failed to process response from device: {e} (Received: {response_data})")
        except Exception as e:
            print(f"[UNEXPECTED ERROR] {e}")
        
        return None