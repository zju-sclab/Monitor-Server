# Monitor-Server V0.2
Monitor-Server of SmartCar, using RVIZ to conduct visualization.

### Protocol
Current Protocol Version is V0.2, as shown below:  
<img src="YunleProtocol/v3.png" width="400" height="450" />

Details could be refered to [vehicle2server_protocol.md](YunleProtocol/vehicle2server.md)

### Feature
> 1. using json format to transport message --for readable
> 2. using twisted-python to conduct async function
> 3. multi-monitor of vehicles is supported
> 4. using rviz to conduct visualization function, both 2D/3D

### TODO
1. Currently, only the monitor of current-pose has been finished.  
2. Multi-monitor of vehicles is supported but never tested.  
3. The effects of visualization should be promoted.

### Attention:
