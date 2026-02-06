import time, re, struct, socket, requests

ROBOT_IP = "192.168.0.11" # to set
URL = f"http://{ROBOT_IP}/MD/CURPOS.DG"

UDP_IP = "127.0.0.1"
UDP_PORT = 5555

POLLING_RATE = 25	# Hz
TIMEOUT = 1.0		# seconds
MAX_LATENCY = 0.25	# seconds
PACK_FMT = '<dd6d'	# timestamp, latency, 6 joints

joint_pattern = re.compile(r'Joint\s+(\d+):\s*([-+]?\d+\.?\d*)', re.IGNORECASE)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
session = requests.Session()

def read_joints():
	t_request = time.time()
	response = session.get(URL, timeout=TIMEOUT)
	t_response = time.time()
	response.raise_for_status()

	joints = [0.0]*6
	found = [False]*6
	for j, v in joint_pattern.findall(response.text):
		i = int(j)
		if 1 <= i <= 6:
			joints[i - 1] = float(v)
			found[i - 1] = True

	t_mid = 0.5 * (t_request + t_response)
	latency = t_response - t_request
	all_found = all(found)

	return t_mid, latency, joints, all_found

def main():
	dt = 1.0 / POLLING_RATE
	print(f"Polling manipulator at {ROBOT_IP} at {POLLING_RATE} Hz, sending UDP to {UDP_IP}:{UDP_PORT}")

	while True:
		try: 
			t, latency, q, found = read_joints()
			if found and latency < MAX_LATENCY:
				packet = struct.pack(PACK_FMT, t, latency, *q)
				sock.sendto(packet, (UDP_IP, UDP_PORT))
		except KeyboardInterrupt:
			print("Exiting...")
			break
		except Exception:
			pass
		
		time.sleep(dt)

if __name__ == "__main__":
	main()