

rostopic pub /udp_node/udp_send udp/Packet "{source:{address: [127, 0, 0, 1]}, sourcePort: 55345, destination:{address: [127,0,0,1]}, destinationPort: 6101, data: [1,2,3,3,2,1,2,3,2,1,2,3,1,2,3,1]}" -r 20
