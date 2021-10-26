import usocket,network

SSID ='mi_t'     # Network SSID
KEY  ='1234567890'     # Network key
HOST =''     # Use first available interface
PORT = 4660  # Arbitrary non-privileged port

wlan = network.WINC()
print("try to connect",SSID)
wlan.connect(SSID, key=KEY)
print(wlan.ifconfig())
print("UDP Binding on 4660")
# Create server socket
s = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
s.bind([HOST, PORT])
s.settimeout(0.8)
dat,addr=s.recvfrom(20)
print(addr)
print(dat)
s.sendto(dat,addr);