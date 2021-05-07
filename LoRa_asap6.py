import simpy
import random
import numpy as np
import math
import sys
import matplotlib.pyplot as plt
import os
from matplotlib.patches import Rectangle
import operator as op
from functools import reduce

totsoftots=0
fdr=0
powerConsumed=0
GPS = 0
relayCircleGlobs = 0
##number of symbols in thecd frame -> 8




for simulation_number in range(10):
    powerConsumed = 0
    # turn on/off graphics
    graphics = 0

    # do the full collision check
    full_collision = False

    # this is an array with measured values for sensitivity
    # see paper, Table 3
    sf7 = np.array([7,-126.5,-124.25,-120.75])
    sf8 = np.array([8,-127.25,-126.75,-124.0])
    sf9 = np.array([9,-131.25,-128.25,-127.5])
    sf10 = np.array([10,-132.75,-130.25,-128.75])
    sf11 = np.array([11,-134.5,-132.75,-128.75])
    sf12 = np.array([12,-137.25,-132.25,-132.25])

    #
    # check for collisions at base station
    # Note: called before a packet (or rather node) is inserted into the list
    #
    def checkcollision(packet):
        col = 0 # flag needed since there might be several collisions for packet
        # lost frames don't collide
        # (they go undetected since the received power is below the receiver sensitivity)
        if packet.lost:
           return 0
        if packetsAtBS[packet.rcvr_id]:
            # see if another frame currently being recieved is capable of causing the loss of the frame
            # do the above for every other frame that are currently being received
            for other in packetsAtBS[packet.rcvr_id]:
                if other.id != packet.nodeid:
                    if packet.freq == other.packet[packet.rcvr_id].freq:
                        col = timingandpowercheck(packet, other.packet[packet.rcvr_id])
            return col
        return 0

    #
    # this function checks if the freshly arrived frame (p1) is lost due to a frame p2 that
    # was already being received by the gateway. Returns 1 if frame loss conditions are satisfied
    #

    ###??

    def timingandpowercheck(p1, p2):
        #
        # Is p1 (which just arrived) lost due to p2 (which arrived earlier)?
        #

        # Capture-effect thresholds
        if (p1.sf == p2.sf):
            powerThreshold = 6 # dB
        else:
            powerThreshold = -1000 # dB

        # Is p2 a strong inteferer from p1's perspective?
        if (p1.rssi - p2.rssi) < powerThreshold:
            p2_is_strong_interferer = 1
        else:
            p2_is_strong_interferer = 0

        # keeping track of stong interference events (for offline analysis / debugging)
        if p1.nodeid == 0:
            total_int[p2.sf - 7] = total_int[p2.sf - 7] + 1
            if p2_is_strong_interferer:
                strong_int[p2.sf - 7] = strong_int[p2.sf - 7] + 1

        # If p2 is a strong interferer, then p1 is lost unless p2
        # ends before the first 3 preamble symmbols of p1.
        # If p2 is a weak interferer, then same condition applies,
        # but in addition, the gateway must have already
        # synchronized to p2.

        # no. of preamble symbols in a frame
        Npream = 8

        ## --???


        # we can lose at most (Npream - 5) * Tsym of p1's preamble
        Tpreamb = 2**p1.sf/(1.0*p1.bw) * (Npream - 5)

        # the time at which p2 will end
        p2_end = p2.addTime + p2.rectime

        # ending time for the critical section of p1
        p1_cs = env.now + Tpreamb

        # has the gateway already synchronized to p2?
        preamble_duration = 2**p2.sf/(1.0*p2.bw) * Npream
        time_needed_to_sync = 2**p2.sf/(1.0*p2.bw) * 5
        clean_preamble_duration = min(env.now, p2.addTime + preamble_duration)  - p2.interf_end_time
        if (p2.synchronization_failure == 0) and (clean_preamble_duration > time_needed_to_sync):
            p2_syncd = 1
        else:
            p2_syncd = 0

        # check whether p2 corrupts p1's critical section
        if p1_cs < p2_end:
            # p1 collided with p2 and lost
            if (p2_is_strong_interferer == 1) or (weak_interference == 1 and p2_is_strong_interferer == 0 and p1.sf == p2.sf and p2_syncd):
                p1.collided = 1
                p1.synchronization_failure = 1

        else:
            if (p2_is_strong_interferer == 1) or (weak_interference == 1 and p2_is_strong_interferer == 0 and p1.sf == p2.sf and p2_syncd):
                p1.interf_end_time = p2_end

        #
        # Is p2 lost due to p1?
        #

        # Is p1 a strong inteferer from p2's perspective?
        if (p2.rssi - p1.rssi) < powerThreshold:
            p1_is_strong_interferer = 1
        else:
            p1_is_strong_interferer = 0

        # If p1 is a strong interferer, then p2 is lost no matter how much
        # the two packets overlap.
        # If p1 is a weak interferer, it can't result in the loss of p2.
        if (p1_is_strong_interferer == 1):
            p2.collided = 1

        # keeping track of stong interference events (for offline analysis / debugging)
        if p2.nodeid == 0:
            total_int[p1.sf - 7] = total_int[p1.sf - 7] + 1
            if p1_is_strong_interferer:
                strong_int[p1.sf - 7] = strong_int[p1.sf - 7] + 1

        return p1.collided


    #
    # this function computes the airtime of a packet
    # according to LoraDesignGuide_STD.pdf
    #




    ## sf->spreading factor
    ## cr--coding rate
    ## pl->payload lenght
    ## bw->bandwidth

    def airtime(sf,cr,pl,bw):
        H = 0        # implicit header disabled (H=0) or not (H=1)
        DE = 0       # low data rate optimization enabled (=1) or not (=0)
        Npream = 8   # number of preamble symbol (12.25  from Utz paper)
        if bw == 125 and sf in [11, 12]:
            # low data rate optimization mandated for BW125 with SF11 and SF12
            DE = 1
        if sf == 6:
            # can only have implicit header with SF6
            H = 1

        Tsym = (2.0**sf)/bw
        Tpream = (Npream + 4.25)*Tsym
        payloadSymbNB = 8 + max(math.ceil((8.0*pl-4.0*sf+28+16-20*H)/(4.0*(sf-2*DE)))*(cr+4),0)   ##-----??
        Tpayload = payloadSymbNB * Tsym
        return Tpream + Tpayload

    #
    ## this function allocates a spreading factor to a node based on the gateway-node distance
    #
    def allocate_SF(distance):
        rx_pow = Ptx - Lpld0 - 10*gamma*math.log10(distance/d0)
        if rx_pow > sensi[0,1]:
            return 7
        elif rx_pow > sensi[1,1]:
            return 8
        elif rx_pow > sensi[2,1]:
            return 9
        elif rx_pow > sensi[3,1]:
            return 10
        elif rx_pow > sensi[4,1]:
            return 11
        else:
            return 12


    #
    # this function creates a BS
    #
    class myBS():
        def __init__(self, id):
            self.id = id
            self.x = 0
            self.y = 0
            self.dist = 0
            self.sf=0
            if (self.id == 0):
                self.x = 0
                self.y = 0
                self.dist = 0
            else:
                if (relayCircle==0):
                    theta = random.uniform(0, 2*3.14)                              ##randomly allocating the base station
                    self.dist = random.uniform(0, relay_radius)
                    self.x = self.dist * math.cos(theta)
                    self.y = self.dist * math.sin(theta)
                    self.sf=allocate_SF(self.dist)
                else:
                    theta = self.id * 2*3.14/numRelays
                    self.dist = random.uniform(relay_radius, relay_radius+relay_radius/10)
                    self.y = self.dist * math.sin(theta)
                    self.x = self.dist * math.cos(theta)
                    self.sf=allocate_SF(self.dist)

            # Buffer contents at BS
            self.buffered_node = []
            self.buffered_seq = []

            # print(self.x,self.y,self.dist)
            plt.scatter(self.x,self.y,color="green",marker="x",)
            plt.text(self.x,self.y,self.sf)



    #
    #this function creates a relay (not currently in use; needs more work)
    #currently bs[0], bs[1],...,bs[num_gateways - 1] are the gateways
    #and bs[num_gateways], bs[num_gateways + 1],..., bs[num_gateways + num_relays - 1] are the relays
    #TODO: develop the myRelay class so that it can be used to create relays
    #


    class myRelay():
        def __init__(self, id):
            self.id = id
            self.x = 0
            self.y = 0

            print("this is not being used")
            # theta = random.uniform(0, 2*3.14)
            self.x = random.uniform(10,20)                #randomly providing the relay location between 10 and 20
            self.y = random.uniform(10,20)

            # Buffer contents at BS
            self.buffered_node = []
            self.buffered_seq = []

    #
    # this function creates a node
    #



    class myNode():
        def __init__(self, id, period):
            global bs

            self.id = id
            self.period = period
            self.packet = []
            self.dist = []
            self.frame_seqnum = -1

            # node 0 is at a fixed distance from the origin as specified by input parameter node0_dist
            # other nodes are placed within a circle of radius = circle_radius uniformly at random
            if self.id == 0:
                self.dist = node0_dist
            else:
                self.dist = random.uniform(0, circle_radius)         #plotting the nodes

            theta = random.uniform(0, 2*3.14)

            # x- and y-coordinate of the node
            self.x = self.dist * math.cos(theta)                   #randmly giving the nodes location
            self.y = self.dist * math.sin(theta)

            # determine what spreading factor the node should use for its frames
            if experiment == 0:
                self.sf = sensor_sf
            else:
                if basedOnRelayCircle == 1 and self.dist > relay_radius:
                    self.sf=allocate_SF(self.dist-relay_radius)
                else:
                    self.sf = allocate_SF(self.dist)
                # print(self.sf)

            # intialize counters to keep track of how many measurements the node has sent
            # and how many have been successfully delivered
            self.sent = 0
            self.delivered = 0

            plt.scatter(self.x,self.y,color="red",marker=".")
            # plt.annotate(self.x,self.y,self.sf)
            plt.text(self.x,self.y,self.sf)


    #
    # this function creates a packet (associated with a node)
    #
    class myPacket():
        def __init__(self, nodeid, plen, distance, rcvr_id,nodeDist):
            global experiment
            global Ptx
            global gamma
            global d0
            global Lpld0
            global coding_rate
            global bandwidth

            # we are defining a frame here

            # new: base station ID
            self.rcvr_id = rcvr_id
            self.seqNr=0
            # originating node id
            self.nodeid = nodeid

            # spreading factor, code rate, and bandwidth for the frame
            self.sf = nodes[nodeid].sf
            self.cr = coding_rate
            self.bw = bandwidth
            self.dist=nodeDist
            # payload length
            self.pl = plen
            # print(plen)
            # propagation loss for the frame
            propagation_loss = Lpld0 + 10*gamma*math.log10(distance/d0)   ####lpld0 & d0 are pathloss parameter


            # nakagami is a parameter to model model the fading on the links (If 0, then no fading; otherwise, use a value of 0.5 or greater)

            # fading gain for the frame
            if (nakagami_m == 0):
                fading_gain = 0;
            else:
                fading_gain = 10 * math.log10(random.gammavariate(nakagami_m, 1.0/float(nakagami_m)))

            # received power in the frame
            self.rssi = Ptx - propagation_loss + fading_gain

            # Ptx power of the transmitted signal

            # choose one of the three center frquencies at random
            self.freq = random.choice([860000000, 864000000, 868000000])

            # on-air time for the frame
            self.rectime = airtime(self.sf,self.cr,self.pl,self.bw)

            # denote if packet is collided
            self.collided = 0
            self.processed = 0
            self.synchronization_failure = 0
            self.interf_end_time = 0

            # mark the packet as lost if the received power is below the receiver sensitivity
            self.lost = self.rssi < sensi[self.sf - 7,1]


    #
    # main discrete event loop, runs for each node
    # a global list of packet being processed at the gateway is maintained
    #
    def transmit(env,node):
        while True:

    # nrBs->totol number of basestations
            global packetSeq
            global nrBS,  t_r, t_t, t_s

            # payload size for the frame
            if node.id == 0:
                payload_size = node0_payload_bytes
            else:
                payload_size = random.randrange(min_payload_bytes, max_payload_bytes + 1)

            # transmission_type = 1 for exponentially distributed packet intervals, 2 for periodic transimssions
            if transmission_type == 1:
                yield env.timeout(random.expovariate(1.0/float(node.period)))
            else:
                if node.sent == 0:
                    yield env.timeout(random.uniform(0, avgSendTime))
                yield env.timeout(node.period - airtime(node.sf,coding_rate,payload_size,bandwidth))

            # increment the send counter for the node
            node.sent = node.sent + 1

            # increment the global sequence number for the frame
            packetSeq = packetSeq + 1

            # increment the local sequence number for the frame at the sending node
            node.frame_seqnum = node.frame_seqnum + 1

            # create the packet
            node.packet = []

            for rcvr_id in range(0, nrBS):
                dist = np.sqrt((node.x-bs[rcvr_id].x)*(node.x-bs[rcvr_id].x)+(node.y-bs[rcvr_id].y)*(node.y-bs[rcvr_id].y))
                dist2=np.sqrt((node.x)*(node.x)+(node.y)*(node.y))
                node.packet.append(myPacket(node.id, payload_size, dist, rcvr_id,dist2))


            # packet reception at gateway
            for rcvr_id in range(0, numGateways):

    	    # a gateway is always receiving
                rx_flag = 1

                if (node in packetsAtBS[rcvr_id]):
                    print ("ERROR: packet already in")
                else:
                    # adding packet if no collision
                    collision_flag = checkcollision(node.packet[rcvr_id])
                    if collision_flag == 1 or rx_flag == 0:
                        #print (rcvr_id, collision_flag, rx_flag)
                        node.packet[rcvr_id].collided = 1
                    else:
                        node.packet[rcvr_id].collided = 0
                    packetsAtBS[rcvr_id].append(node)
                    node.packet[rcvr_id].addTime = env.now
                    node.packet[rcvr_id].interf_end_time = env.now
                    node.packet[rcvr_id].seqNr = packetSeq


    # if (node.packet[rcvr_id].dist < bs[rcvr_id].dist):
    #packet reception at relays
            for rcvr_id in range(numGateways, nrBS):
                # set rx_flag to 1 if the incoming frame fits within the relay's receive window
                x = (env.now - (rcvr_id - numGateways) * t_t) % (t_r + t_t + t_s)
                rx_flag =(node.packet[0].rectime < t_r - x)
                flagg=0
                global powerConsumed
                # print(rcvr_id, "  ",node.packet[rcvr_id].dist)
                if gpsFlag == 1:
                    # print("GPS is ON")
                    if (alpha*bs[rcvr_id].dist) > node.packet[rcvr_id].dist :
                        # print(alpha)
                        flagg = 1


                if (node in packetsAtBS[rcvr_id]):
                    print ("ERROR: packet already in")
                else:
                    # adding packet if no collision
                    collision_flag = checkcollision(node.packet[rcvr_id])
                    if collision_flag == 1 or rx_flag == 0:
                        node.packet[rcvr_id].collided = 1
                    elif flagg == 1:
                        # print("Not reaching")
                        node.packet[rcvr_id].collided = 1
                        # print("DC")
                    else:
                        node.packet[rcvr_id].collided = 0
                        # print(powerConsumed)
                        # print(payload_size)
                        # print(random.randrange(min_payload_bytes, max_payload_bytes + 1))
                        # print(node.packet[rcvr_id].pl)
                        # print(airtime(bs[rcvr_id].sf,node.packet[rcvr_id].cr,node.packet[rcvr_id].pl,node.packet[rcvr_id].bw))
                        # print(pow(2, bs[rcvr_id].sf))




                    packetsAtBS[rcvr_id].append(node)
                    node.packet[rcvr_id].addTime = env.now
                    node.packet[rcvr_id].interf_end_time = env.now
                    node.packet[rcvr_id].seqNr = packetSeq



            # packet reception time
            yield env.timeout(airtime(node.sf,coding_rate,payload_size,bandwidth))

            # if packet did not collide, add it to the list of received packets
            # unless it is already in
            for rcvr_id in range(0, nrBS):
                # print(node.packet[rcvr_id].synchronization_failure)
                if node.packet[rcvr_id].lost:
                    lostPackets.append(node.packet[rcvr_id].seqNr)
                else:
                    if node.packet[rcvr_id].collided == 0:
                        if (rcvr_id == 0):
                            node.delivered = node.delivered + 1
                        packetsRecBS[rcvr_id].append(node.packet[rcvr_id].seqNr)

                        # note the seq. no. and the sender
                        if rcvr_id < numGateways:
                            j = max(0, node.frame_seqnum - redundant_readings)
                            for i in range(j, node.frame_seqnum + 1):
                                gateway_contents[node.id][i] = 1
                        else:
                            relay_buffer_nodeid[rcvr_id].append(node.id)
                            relay_buffer_seqnum[rcvr_id].append(node.frame_seqnum)
                            payloadarray[rcvr_id].append(node.packet[rcvr_id].pl)
                            # print(payloadarray[rcvr_id])

                        if (recPackets):
                            if (recPackets[-1] != node.packet[rcvr_id].seqNr):
                                recPackets.append(node.packet[rcvr_id].seqNr)
                        else:
                            recPackets.append(node.packet[rcvr_id].seqNr)
                            # print(node.packet[rcvr_id].seqNr)
                            # print(rcvr_id)
                    else:
                        # XXX only for debugging
                        # print(node.packet[rcvr_id].pl)
                        collidedPackets.append(node.packet[rcvr_id].seqNr)
                        # print(rcvr_id)

            # complete packet has been received by base station
            # can remove it
            for rcvr_id in range(0, nrBS):
                if (node in packetsAtBS[rcvr_id]):
                    packetsAtBS[rcvr_id].remove(node)
                    # reset the packet
                    node.packet[rcvr_id].collided = 0
                    node.packet[rcvr_id].processed = 0
                    node.packet[rcvr_id].synchronization_failure = 0
                    node.packet[rcvr_id].interf_end_time = 0



    #
    #this function simulates packet transmission from the relays to gateway-0
    #recall that bs[0], bs[1],...,bs[num_gateways - 1] are the gateways
    #and bs[num_gateways], bs[num_gateways + 1],..., bs[num_gateways + num_relays - 1] are the relays
    #TODO-1: make a separate class for the relays (a "myRelay()" class placeholder has been created)
    #TODO-2: receive relay transmissions at all gateways, not just gateway-0
    #TODO-3: incoprorate interference in frame receptions over relay-gateway links
    #
    def relay_tx(env, bs_id):
        while True:
            global bs, BS_start, powerConsumed
            if BS_start[bs_id] == 1:
                BS_start[bs_id] = 0
                yield env.timeout(i*t_t)

            yield env.timeout(t_r + t_t + t_s)
            # print(bs_id)

            # transmit the buffered contents to gateway-0 (bs_id = 0)
            if (len(relay_buffer_seqnum[bs_id]) > 0):
                # print(relay_payload_bytes)
                # print(relay_payload_bytes)
                # print(bs_id,sum(payloadarray[bs_id]))
                if (sum(payloadarray[bs_id]) > relay_payload_bytes):
                    if(relay_payload_bytes!=0):
                        powerConsumed+=airtime(bs[bs_id].sf,1,relay_payload_bytes,125)
                else:
                    powerConsumed+=airtime(bs[bs_id].sf,1,sum(payloadarray[bs_id]),125)

                # print(len(relay_buffer_nodeid[bs_id])*len(relay_buffer_seqnum[bs_id]))
                # print(relay_buffer_seqnum[bs_id])
                # print(len(relay_buffer_seqnum[bs_id]))

                # print(airtime(bs[bs_id]))
                # distance between the relay and gateway-0
                d = np.sqrt((bs[0].x-bs[bs_id].x)*(bs[0].x-bs[bs_id].x)+(bs[0].y-bs[bs_id].y)*(bs[0].y-bs[bs_id].y))

                # propagation loss calculation
                propagation_loss = Lpld0 + 10*gamma*math.log10(d/d0)

                # fading gain
                if (nakagami_m == 0):
                    fading_gain = 0;
                else:
                    fading_gain = 10 * math.log10(random.gammavariate(nakagami_m, 1.0/float(nakagami_m)))


                rssi = Ptx - propagation_loss + fading_gain
                ##rssi is the power of the frame?

                # was the relay's frame successfully received by gateway-0?
                tx_success = rssi > sensi[relay_sf - 7,2]

                # after a successful recetption, store the relay's payload to the gateway
                if (tx_success):
                    # global powerConsumed
                    # if relay_payload_bytes!=0:
                        # print(relay_payload_bytes)
                        # powerConsumed+=airtime(bs[bs_id].sf,1,relay_payload_bytes,125)
                    idx_list = []
                    for j in range(0, len(relay_buffer_seqnum[bs_id])):
                        idx_list.append(0)
                    if relay_payload_bytes >= len(relay_buffer_seqnum[bs_id]):
                        for j in range(0, len(relay_buffer_seqnum[bs_id])):
                            idx_list[j] = 1
                    else:
                        m = 0
                        while True:
                            k = random.randint(0, len(relay_buffer_seqnum[bs_id]) - 1)
                            if idx_list[k] == 0:
                                idx_list[k] = 1
                                m = m + 1
                            if m >= relay_payload_bytes:
                                break

                    for x in range(0, len(relay_buffer_seqnum[bs_id])):
                        if idx_list[x] == 1:
                            # print(gateway_contents[relay_buffer_nodeid[bs_id][x]][relay_buffer_seqnum[bs_id][x]])
                            gateway_contents[relay_buffer_nodeid[bs_id][x]][relay_buffer_seqnum[bs_id][x]] = 1

            # clear the buffer
            relay_buffer_nodeid[bs_id] = []
            relay_buffer_seqnum[bs_id] = []
            payloadarray[bs_id] = []
            if bs_id == -1:
                print ("buff_clear")


    #
    # "main" program
    #

    # get arguments
    if len(sys.argv) == 30:
        nrNodes = int(sys.argv[1])  # No. of sensors in the network
        avgSendTime = int(sys.argv[2]) # Mean of the time interval between two consecutive packets from a sensor
        experiment = int(sys.argv[3]) # Defines certain radio settings
        simtime = int(sys.argv[4]) # No. of milliseconds of network operation to be simulated
        numRelays = int(sys.argv[5]) # Number of relays
        redundant_readings = int(sys.argv[6]) # Let redundant_readings = r. Then frame i contains the sensor readings i, i-1, ..., i-r. (See EW and Globecom papers)
        sensor_sf = int(sys.argv[7]) # Spreading factor used by the sensors for experiment = 0 (distance-based assignment is used if experiment > 0)
        relay_sf = int(sys.argv[8]) # Spreading factor used by the relays
        nakagami_m = float(sys.argv[9]) # Nakagami m parameter to model the fading on the links (If 0, then no fading; otherwise, use a value of 0.5 or greater)
        transmission_type = int(sys.argv[10]) # 1 = exponentially distributed packet interval; 2 = periodic transmissions
        numGateways = int(sys.argv[11]) # Number of gateways
        circle_radius = int(sys.argv[12]) # Radius of the circular region in meters
        node0_dist = int(sys.argv[13]) # Distance between the gateway and the desired sender
        node0_payload_bytes = int(sys.argv[14]) # Payload bytes in a frame from the desired sender
        min_payload_bytes = int(sys.argv[15]) # Minimum possible payload bytes from an arbitray sender
        max_payload_bytes = int(sys.argv[16]) # Maximum possible payload bytes from an arbitray sender
        coding_rate = int(sys.argv[17]) # Coding rate for the frames
        bandwidth = int(sys.argv[18]) # Transmission bandwidth
        Ptx = int(sys.argv[19]) # power of the transmitted signal in dBm
        gamma = int(sys.argv[20]) # Path loss exponent
        weak_interference = int(sys.argv[21]) # Set to 1 if a weak intefering frame can cause frame loss
        relay_radius = int(sys.argv[22]) # The relays are randomly distributed over a circle
        t_r = int(sys.argv[23]) # Relay reception time (ms)
        t_t = int(sys.argv[24]) # Relay transmission time (ms)
        t_s = int(sys.argv[25]) # Relay sleep time (ms)
        relayCircle = int(sys.argv[26])
        gpsFlag = int(sys.argv[27])
        alpha = float(sys.argv[28])
        basedOnRelayCircle = int(sys.argv[29])
    else:
        print(len(sys.argv))
        print ("usage: ./loraDir <nodes> <avgsend> <experiment> <simtime> <basestation>")
        exit(-1)

    nrBS = numRelays + numGateways # We refer to both gateways and relays as base stations.

    # Use the capture-effect based interference model
    full_collision = 1

    # global stuff
    nodes = []
    packetsAtBS = []

    env = simpy.Environment()

    relay_buffer_nodeid = []
    relay_buffer_seqnum = []
    payloadarray= []
    BS_start = []

    gateway_contents = []


    ##--????

    # max_readings is the maximum number of receptions from a sensor from a sensor that the simulation can handle
    max_readings = 5*int(simtime/avgSendTime)

    # gateway_contents is a matrix that will keep track of which measurements are received from each node. All elements of gateway_contents are intialized to zero. The element in row i and column j is set 1 when the j-th measurement from the i-th sensor is received
    for i in range(0, nrNodes):
        gateway_contents.append([0]*(max_readings-1))


    # relays' tx and rx windows
    # a relay recieves for t_r ms, transmits for t_t ms, and then sleeps for t_s ms. This cycle continues until the end of the simulation run.
    # t_r = avgSendTime
    # t_t = avgSendTime * 0.01
    # t_s = t_t*100

    # max. no. of bytes a relay packet can carry

    ##when our current airtime is greater than the
    relay_payload_bytes = 1;
    while airtime(relay_sf, 1, relay_payload_bytes, 125) < t_t :
        relay_payload_bytes = relay_payload_bytes + 1
        # print(relay_payload_bytes)
        # powerConsumed+=airtime(relay_sf,1,relay_payload_bytes,125)

    if airtime(relay_sf, 1, relay_payload_bytes, 125) > t_t :
        relay_payload_bytes = relay_payload_bytes - 1

    # a relay uses only half its payload for data; the rest is used to indicate the sensor IDs whose readings are forwarded
    # TODO: incorporate a more generic calculation based on the transmission strategy
    relay_payload_bytes = int(relay_payload_bytes/2)

    # max distance: 300m in city, 3000 m outside (5 km Utz experiment)
    # also more unit-disc like according to Utz
    nrCollisions = 0 # Will be incremented by 1 whenever a collision occurs
    nrReceived = 0 # Will be incremented by 1 whenever the gateway receives a frame
    nrProcessed = 0
    sf_usage = np.array([0,0,0,0,0,0]) # to keep track of how many packets used a certain spreading factor
    strong_int = np.array([0,0,0,0,0,0]) # to keep track of how many packets using a certain spreading factor resulted in strong interference
    total_int = np.array([0,0,0,0,0,0]) # to keep track of how many packets using a certain spreading factor resulted in interference
    packetSeq = 0 # Will be incremented by 1 whenever a frame is sent by any sensor

    # list of received packets
    recPackets=[]
    collidedPackets=[]
    lostPackets = []

    # path-loss related parameters. Refer to the paper "Do LoRa Low-Power Wide-Area Networks Scale?" by M. Bor et al.
##check the paper mentioned for d0 and Lpdld0
    d0 = 40.0
    Lpld0 = 127.41


    # sensi is an array of the receiver sensitivities for different spreading factors
    # receiver sensitivity is the minimum power that signal arriving at the gateway must possess for the frame to be correctly received
    sensi = np.array([sf7,sf8,sf9,sf10,sf11,sf12])

    # maximum number of packets a receiver can receive at the same time
    # this depends on the number of demodulation paths the receiver has
    maxBSReceives = 1000

    # list of base stations
    bs = []

    # list of packets at each base station, init with 0 packets
    packetsAtBS = []
    packetsRecBS = []

    # a record of the stored frames at the relays
    relay_buffer_nodeid = []
    relay_buffer_seqnum = []


    # activate the gateways and the relays
    for i in range(0,nrBS):
        b = myBS(i)
        bs.append(b)
        packetsAtBS.append([])
        packetsRecBS.append([])
        relay_buffer_nodeid.append([])
        relay_buffer_seqnum.append([])
        payloadarray.append([])
        BS_start.append(1)
        #activate the relays
        if i > 0:
            env.process(relay_tx(env,i))

    # activate the nodes
    for i in range(0,nrNodes):
        node = myNode(i, avgSendTime)
        nodes.append(node)
        env.process(transmit(env,node))

    # store nodes and basestation locations
    # with open('nodes.txt', 'w') as nfile:
    #     for node in nodes:
    #         nfile.write('{x} {y} {id}\n'.format(**vars(node)))
    #         sf_usage[node.sf - 7] = sf_usage[node.sf - 7] + 1
    #         #print("dist: ", node.dist, " sf: ", node.sf)
    #
    # with open('basestation.txt', 'w') as bfile:
    #     for basestation in bs:
    #         bfile.write('{x} {y} {id}\n'.format(**vars(basestation)))

    # start simulation
    env.run(until=simtime)


    ##calculating the fdr rates from the

    total_sent = 0
    total_rcvd = 0
    total_rcvd_relay = 0
    for i in range(0, nrNodes):
        total_sent = total_sent + nodes[i].sent
        total_rcvd = total_rcvd + nodes[i].delivered
        total_rcvd_relay = total_rcvd_relay + sum(gateway_contents[0])

    rx_rate = float(total_rcvd)/float(total_sent)

    rx_rate_0 = float(nodes[0].delivered)/float(nodes[0].sent)

    rx_rate_cumulative_relay = float(total_rcvd_relay)/float(total_sent);
    # rx_rate_0_relay = float(sum(gateway_contents[0]))/float(nodes[0].sent)
    # rx_rate_1_relay = float(sum(gateway_contents[1]))/float(nodes[1].sent)
    for i in range(0,nrNodes):
        rx_rate_relay = float(sum(gateway_contents[i]))/float(nodes[i].sent)
        fdr+=rx_rate_relay
        # print("Distance: ", nodes[i].dist, "SF: ", nodes[i].sf, "Nodes: ", nrNodes, "FDR Node : ", i, " : ", rx_rate_relay)

    #plt.show()


    #print("Distance: ", nodes[0].dist, "SF: ", nodes[0].sf, "Nodes: ", nrNodes, "FDR (node 0): ", rx_rate_0_relay)
    #print("Distance: ", nodes[1].dist, "SF: ", nodes[1].sf, "Nodes: ", nrNodes, "FDR (node 0): ", rx_rate_1_relay)

    # print(fdr/nrNodes)

    # print(sensorPowerConsumed)
    # totsoftots+=((fdr/nrNodes))
    #
    # if simulation_number == 0:
    #     if gpsFlag == 1:
    #         print("GPS is on")
    #     else :
    #         print("GPS is off")
    #     if relayCircle == 1:
    #         print("relayCircle is on")
    #     else :
    #         print("relayCircle is off")

    # tot_fdr_rate+=rx_rate_0_relay
    # plt.show()

    power_consumed=open("powerConsumedasap6.txt","a")
    power_consumed.write(str(circle_radius))
    power_consumed.write(',')
    power_consumed.write(str(powerConsumed))
    power_consumed.write('\n')
    power_consumed.close()

    fdrConsumed=open("fdrRateasap6.txt","a")
    fdrConsumed.write(str(circle_radius))
    fdrConsumed.write(',')
    fdrConsumed.write(str( (fdr/nrNodes)*100) )
    fdrConsumed.write('\n')
    fdrConsumed.close()

    fdr = 0
    # for i in range(nrNodes):
    #     total_sent = 0
    #     total_rcvd = 0
    #     total_rcvd_relay = 0
    #     for i in range(0, nrNodes):
    #         total_sent = total_sent + nodes[i].sent
    #         total_rcvd = total_rcvd + nodes[i].delivered
    #         total_rcvd_relay = total_rcvd_relay + sum(gateway_contents[i])
    #
    #     rx_rate = float(total_rcvd)/float(total_sent)
    #
    #     rx_rate_0 = float(nodes[i].delivered)/float(nodes[i].sent)
    #
    #     rx_rate_cumulative_relay = float(total_rcvd_relay)/float(total_sent);
    #     rx_rate_relay = float(sum(gateway_contents[i]))/float(nodes[i].sent)
    #
    #
    #     print("Distance: ", nodes[i].dist, "SF: ", nodes[i].sf, "Nodes: ", nrNodes, "FDR (node 0): ", rx_rate_relay)
    # tot_fdr_rate+=rx_rate_0_relay

    # plt.show()
    # save data into a file

    # res = str(nodes[0].delivered) + " " + str(nodes[0].sent) + "\n"

    # fname = "Dist" + str(node0_dist) + "_PL" + str(node0_payload_bytes) + "_N" + str(nrNodes) + "_WI" + str(weak_interference) + ".txt"
    # f = open(fname, "a")
    # f.write(res)
    # f.close()

    # fdr_rate_txt_file = open("fdr_rate_txt_file.txt","a")
    # fdr_rate_txt_file.write(str(rx_rate_0_relay))
    # fdr_rate_txt_file.write('\n')
    # fdr_rate_txt_file.close()

# avg_fdr_rate=tot_fdr_rate/simulation_runs

# relay_numbers=open("relayVSsensors_sleep_time_2000.txt","a")
# relay_numbers.write(str(avg_fdr_rate))
# relay_numbers.write('\n')
# relay_numbers.close()

# totsoftots/=100
# print(totsoftots," Distance : ", node0_dist)
# relay_numbers=open("single_node_no_relays.txt","a")
# relay_numbers.write(str(totsoftots))
# relay_numbers.write('\n')
# relay_numbers.close()s

# relay_numbers=open("relav_8_simulation_sleep.txt","a")
# relay_numbers.write(str(avg_fdr_rate))
# relay_numbers.write('\n')
# relay_numbers.close()


# fdrVSrelayradiusGPSText=open("withGPSLessSimsgdrVSrelaydist.txt","a")
# fdrVSrelayradiusGPSText.write(str(totsoftots))
# fdrVSrelayradiusGPSText.write('\n')
# fdrVSrelayradiusGPSText.close()


# n0_dist_text=open("n0_dist_fdr_sleep_time_2000ms.txt","a")
# n0_dist_text.write(str(avg_fdr_rate))
# n0_dist_text.write('\n')
# n0_dist_text.close()
