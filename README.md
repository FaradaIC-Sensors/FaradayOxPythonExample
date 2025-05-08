Python example on how to interact with FaradayOx sensor  over UART from the PC.
USB to UART connector is required.
Change port in the script to appropriate one  ```sensor = SensorDevice(port="COM15", baudrate=9600)```
For serial communication to work: ```pip install pyserial```

![connection](connection.jpg) 
