import socket
import sys
import struct
import time

MCAST_GRP = '224.0.0.142'
WELLKNOWN_NODE_ID = 634
MAX_NODE_ID = 12		# max network size
PORTS_OFFSET = 9000
RCV_UDP_PORT = PORTS_OFFSET + WELLKNOWN_NODE_ID

# Create a UDP socket for receiving (multicast)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Create a UDP socket for sending (to destinations)
sockd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the receiving socket to the port
server_address = ('', RCV_UDP_PORT)
sock.bind(server_address)

# Register to multicast group as listener
mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
print 'sim-mesh-topo.py - server started on address \'%s\' UDP port %s' % server_address

# time printing
time0 = time.time()

while True:
	data, address = sock.recvfrom(4096)
	source_node = address[1] - PORTS_OFFSET    
	t = round(time.time()-time0,3); # show rel time in sec.
	
	print '%10.3f  rcv frame %3i bytes from node %2i' % (t, len(data), source_node)
	#print data

	# send data to other simulated nodes
	if data:
		# Fully connected mesh case
		dest_nodes = range(1,MAX_NODE_ID+1)
		
		# Send radio frame to all destinations that need to receive it
		for n in dest_nodes:
			if n == source_node:
				continue
			dest_address = ('localhost', PORTS_OFFSET + n )
			sent = sockd.sendto(data, dest_address)
