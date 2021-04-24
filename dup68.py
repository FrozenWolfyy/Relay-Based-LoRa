#!/usr/bin/env python2
# -*- coding: utf-8 -*-


for x in range(16):
# specify simulation parameters
    num_nodes_range = [200] # number of nodes in the network
    avg_inter_frame_interval = 10000 # in ms
    exp_num = 1 # defines certain radio settings (set to >0 for distacne based spreading factor assignment)
    simulation_time = 1008000 # in ms
    relays = x # number # of relay nodes
    redunancy = 0 #ignore th150is parameter
    sf_sensors = 10 # spreading factor for the sensor frames (if exp_naum = 0)
    sf_relays = 8 # spreading factor for the relay frames
    naka_m = 1 # the nakagami m parameter (related to fading)
    tx_type = 1 # 1 : exponentially distributed packet intervals; 2 : periodic transmissions
    num_gw = 1 # number of gateways
    circ_rad = 200 # radius of the circle over which the ndoes are randomly distributed (gateway-0 is at the center)
    n0_dist = 80 # distance between node-0 and gateway-0
    n0_payload = 1 # payload size (in bytess) for node-0's frames
    min_payload = 1 # minimium possible payload size (in bytes) for other node's frames
    max_payload = 25 # maximum possible payload size s(in bytes) for other node's frames
    coding_rate = 1 # channel coding rate for the frames
    bandwidth = 125 # transmission bandwidth in KHz
    transmit_power_dBm = 14 # transmit power in dBm
    path_loss_exponent = 2 # determines signal attenuation as it travels from sender to receiver ##gamma
    weak_interference = 1 # set to 1 if a weak frame can cause sync. failure for a strong frame
    relay_radius = 75  # radius of the circle over which the relays are randomly distributed
    relay_reception_time = 500 # in ms
    relay_transmission_time = 500 # in ms
    relay_sleep_time = 7500 # in ms
    relay_circle=1
    gpsFlag=1
    alpha = 1.250
    basedOnRelayCircle = 0

    # perform multiple simulation runs for the parameters specified above
    # each simulation run employs a random realization of the sensor placement

    for num_nodes in num_nodes_range:
        for weak_interference in range(0,1,1):
            # for i in range(simulation_runs):
            import sys;
            sys.argv=[r'LoRa_Relay_v2_dup68.py', str(num_nodes),str(avg_inter_frame_interval),str(exp_num),str(simulation_time),\
                        str(relays), str(redunancy), str(sf_sensors), str(sf_relays), str(naka_m), str(tx_type), str(num_gw),\
                        str(circ_rad), str(n0_dist), str(n0_payload), str(min_payload), str(max_payload), str(coding_rate),\
                        str(bandwidth), str(transmit_power_dBm), str(path_loss_exponent), str(weak_interference), str(relay_radius),\
                        str(relay_reception_time), str(relay_transmission_time), str(relay_sleep_time),str(relay_circle),str(gpsFlag),str(alpha),str(basedOnRelayCircle)];
            exec(open(r'LoRa_Relay_v2_dup68.py').read());
