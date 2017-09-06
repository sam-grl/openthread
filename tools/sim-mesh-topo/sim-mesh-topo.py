import socket
import sys
import struct

MCAST_GRP = '224.0.0.142'
WELLKNOWN_NODE_ID = 34
PORTS_OFFSET = 9000
RCV_UDP_PORT = PORTS_OFFSET + WELLKNOWN_NODE_ID

# Create a UDP socket for receiving (multicast)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Create a UDP socket for sending (to destinations)
sockd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the receiving socket to the port
server_address = ('', RCV_UDP_PORT)
print >>sys.stderr, 'starting up on %s port %s' % server_address
sock.bind(server_address)
# Register to multicast group as listener
mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

while True:
    #print >>sys.stderr, '\nwaiting to receive message'
    data, address = sock.recvfrom(4096)
    
    print >>sys.stderr, 'rcv %s bytes from %s' % (len(data), address)
    print >>sys.stderr, data
    
    source_node = address[1] - PORTS_OFFSET
    #print >>sys.stderr, 'source node identified: %i' % (source_node)

    if data:
		# Fully connected mesh case
		dest_nodes = range(1,WELLKNOWN_NODE_ID)
		
		# Send radio frame to all destinations that need to receive it
		for n in dest_nodes:
			if n == source_node:
				continue
			dest_address = ('localhost', PORTS_OFFSET + n )
			sent = sockd.sendto(data, dest_address)
		
