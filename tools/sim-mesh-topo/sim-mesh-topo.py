# Simulated network topology - 802.15.4 frame matrix in Python
# Used by ot-cli-*.exe simulated Thread nodes (Posix platform)
#
#  Copyright (c) 2017, Esko Dijk. All rights reserved. 
#  Redistribution and use in source and binary forms is as indicated in the License section of README.md.
#

import socket
import sys
import struct
import time

# Constants and configuration
MCAST_GRP = '224.0.0.142'	# as defined in posix radio.c
WELLKNOWN_NODE_ID = 634		# as defined in posix radio.c
MAX_NODE_ID = 96			# max network size
PORTS_OFFSET = 9000
RCV_UDP_PORT = PORTS_OFFSET + WELLKNOWN_NODE_ID

def matrixAllIsolated(src):
	"isolated - no connectivity for any node"
	return []
	
def matrixFullConn(src):
	"full connectivity matrix"
	dst = range(1,MAX_NODE_ID+1)	# send to all peers
	return dst

def matrixLineTopo(src):
	"line topology 1 - 2 - 3 - 4 ... - LastNode"
	if src==1:
		dst = [src+1]
	elif src == MAX_NODE_ID:
		dst = [MAX_NODE_ID - 1]
	else:
		dst = [src-1,src+1]
	return dst

# Matrix/Topology selection - put in the function ref here and restart script.
MATRIX = matrixLineTopo

def main():
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
		
		print '%10.3f  rcv %3i bytes from %2i ->{' % (t, len(data), source_node), 

		# send data to other simulated nodes
		if data:			
			dest_nodes = MATRIX(source_node)	# for the network matrix, determine who receives the frame
			
			# Send radio frame to all destinations that need to receive it
			for n in dest_nodes:
				if n == source_node:
					continue
				dest_address = ('localhost', PORTS_OFFSET + n )
				sent = sockd.sendto(data, dest_address)
				print "%i" % n,
		print "}"
#-------Script entry point--------
main()
