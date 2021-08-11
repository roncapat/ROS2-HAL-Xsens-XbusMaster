# ROS2 HAL for Xsens XbusMaster (XM-B-XB3)
ROS2 Hardware Abstraction Layer (HAL) for Xsens XBus Master (XM-B-XB3).

# Note on Xbus Master LED and button

The current state is viewed using a specific flash sequence of the LED:
- off for power down
- solid for configuration state
- two short flashes for measurement state waiting for trigger 
- one flash for measurement state sending data

The current mode is visualized using a specific colour of LED: 
- green for serial mode
- blue for Bluetooth mode
- purple for host not found
- yellow for low battery mode 
- red for fault mode

The push button controls the power state of the Xbus Master:
- press once to switch the Xbus Master on
- press 3 consecutive times to switch it off 

# Note on Bluetooth configuration

Pair the device, using 0000 as code if requested.
On linux, use 
```
sudo rfcomm bind xmaster <BT MAC Address>
```
to associate a tty (/dev/rfcomm0) to the device.

Do not connect to the device, just pair it. 
When the node starts and tries to open the tty, the connection will be established and kept open until shutdown of the node.
