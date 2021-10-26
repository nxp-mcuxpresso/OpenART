import usocket,network

SSID ='mi_t'     # Network SSID
KEY  ='1234567890'     # Network key
HOST =''     # Use first available interface
PORT = 4660  # Arbitrary non-privileged port

wlan = network.WINC()
print("try to connect",SSID)
wlan.connect(SSID, key=KEY)
print(wlan.ifconfig())

# Create server socket
s = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
s.bind([HOST, PORT])
s.listen(10)
print("wait for clients")
client,addr=s.accept()
print(addr)
client.settimeout(0.3)
client.send("helo")
print("recv ...")
dat = s.recv(400)
print(dat)