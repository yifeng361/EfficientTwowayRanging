# EfficientTwowayRanging
This code implements efficient multi-node two way ranging of DW1000 in Arduino. In this scenario, there are multiple nodes, who are all trying to perform ranging with other nodes. This code use broadcasting of POLL and FINAL message to save transmission time. 

# Quick introduction
Typically, a complete two way ranging process require 3 message: POLL, RESP and FINAL. An initiator sends the POLL, then the responder receives it and sends the RESP, then the initiator receives the RESP and sends the FINAL. When there are multiple nodes who all want to know its distance from others, this process is time consuming. An efficient method is instead of doing a complete two way ranging between every pair, we can  leverage the "broadcasting" nature of the wireless signal.

For example, say there are 3 nodes in the environment: N1, N2, N3 and assume N1 firstly tries to initialise a ranging.

Traditional two way ranging: N1 sends the POLL to N2. N2 receives it and sends the RESP. N1 receives the RESP and sends the FINAL. N1 sends the POLL to N3. N3 receives it and sends the RESP. N1 receives the RESP and sends the FINAL. 6 packets in all.

Efficient two way ranging: N1 broadcasts the POLL. N2 and N3 receive it and send their RESPs respectively in sequence. When N1 receives all the RESPs from N2 and N3, N1 will send the FINAL. 4 packets in all.


Note that this code assumes:
- Each nodes knows the maximum number of nodes in the network.
- Each node has an ID.

 
- [x] 3-node network is tested. 6/29/2021
- [x] 5-node network is tested. 7/5/2021


If you have any problems, you can contact me via ycao361@gatech.com.
