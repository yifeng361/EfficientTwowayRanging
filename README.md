# EfficientTwowayRanging
This code implements efficient multi-node two way ranging of DW1000 in Arduino. In this scenario, there are multiple nodes, who are all trying to perform ranging with other nodes. This code use broadcasting of POLL and FINAL message to save transmission time. 

This code assumes:
- Each nodes knows the maximum number of nodes in the network.
- Each node has an ID.

 
- [x] 3-node network is tested. 6/29/2021
