"""
Environmental Sensor Communication Module
Provides an interface for communicating with a sensor device via UART.
"""
import serial
import struct
import time
import logging
from enum import IntEnum, auto
from dataclasses import dataclass
from typing import Dict, Any, Optional, Tuple

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("sensor_interface")

# Serial commands
class Command:
    READ = 0xAA
    WRITE = 0x55
    ACK = 0x00
    NEWLINE = 0x0A

# Register map
class Registers(IntEnum):
    # Register map version
    MAP_VER_LSB = 0x00
    MAP_VER_MSB = 0x01
    
    # Status and control registers
    STATUS = 0x02
    CONTROL = 0x04  
    CONFIG = 0x06
    
    # Measurement data registers
    CONCENTRATION = 0x10  # 4 bytes (float)
    TEMPERATURE = 0x14    # 4 bytes (float)
    HUMIDITY = 0x18       # 4 bytes (float)
    
    # Device info registers
    FRONTEND_ID = 0x20    # 4 bytes (uint32)
    FIRMWARE_VER_LSB = 0x24
    FIRMWARE_VER_MSB = 0x25

# Control register commands
class ControlCommands(IntEnum):
    START_MEASUREMENT = 0x01

# Map size
REGISTER_MAP_SIZE = 0x60

@dataclass
class SensorMeasurement:
    """Data class to store sensor measurement results"""
    concentration: float
    temperature: float
    humidity: float
    timestamp: str = ""
    
    def __post_init__(self):
        """Set timestamp if not provided"""
        if not self.timestamp:
            self.timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

class SensorDevice:
    """Interface for the environmental sensor device"""
    
    def __init__(self, port: str, baudrate: int = 9600, timeout: int = 10):
        """
        Initialize the sensor device interface
        
        Args:
            port: Serial port name (e.g., "COM12" or "/dev/ttyUSB0")
            baudrate: Serial communication speed
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self._connect()
    
    def _connect(self) -> None:
        """Establish serial connection to the device"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            logger.info(f"Connected to device on {self.port}")
        except serial.SerialException as e:
            logger.error(f"Failed to connect to device: {e}")
            raise
    
    def _write_command(self, cmd: int, addr: int, value: int) -> bool:
        """
        Write a command to the device
        
        Args:
            cmd: Command code (READ or WRITE)
            addr: Register address
            value: Value to write or length to read
            
        Returns:
            True if command acknowledged, False otherwise
        """
        cmd_bytes = [cmd, addr, value]
        
        try:
            self.ser.write(cmd_bytes)
            if cmd == Command.WRITE:
                # For write commands, expect ACK + newline
                resp = self.ser.read(2)
                return len(resp) == 2 and resp[0] == Command.ACK and resp[1] == Command.NEWLINE
            return True
        except serial.SerialException as e:
            logger.error(f"Serial communication error: {e}")
            return False
    
    def start_measurement(self) -> bool:
        """
        Start a new measurement cycle
        
        Returns:
            True if command was acknowledged, False otherwise
        """
        logger.info("Starting measurement")
        return self._write_command(
            Command.WRITE, 
            Registers.CONTROL, 
            ControlCommands.START_MEASUREMENT
        )
    
    def read_registers(self) -> Dict[str, Any]:
        """
        Read all registers from the device
        
        Returns:
            Dictionary containing parsed register values
        """
        logger.info("Reading registers")
        
        if not self._write_command(Command.READ, 0x00, REGISTER_MAP_SIZE):
            logger.error("Failed to send read command")
            return {}
        
        try:
            # Read full register map + newline
            resp = self.ser.read(REGISTER_MAP_SIZE + 1)
            
            if len(resp) != REGISTER_MAP_SIZE + 1:
                logger.error(f"Incomplete response, got {len(resp)} bytes")
                return {}
            
            # Parse the response
            results = {
                "frontend_id": struct.unpack('<I', resp[Registers.FRONTEND_ID:Registers.FRONTEND_ID+4])[0],
                "reg_map_version": f"{resp[Registers.MAP_VER_MSB]}.{resp[Registers.MAP_VER_LSB]}",
                "firmware_version": f"{resp[Registers.FIRMWARE_VER_MSB]}.{resp[Registers.FIRMWARE_VER_LSB]}",
                "status": resp[Registers.STATUS],
                "control": resp[Registers.CONTROL],
                "config": resp[Registers.CONFIG],
                "measurement": SensorMeasurement(
                    concentration=struct.unpack('<f', resp[Registers.CONCENTRATION:Registers.CONCENTRATION+4])[0],
                    temperature=struct.unpack('<f', resp[Registers.TEMPERATURE:Registers.TEMPERATURE+4])[0],
                    humidity=struct.unpack('<f', resp[Registers.HUMIDITY:Registers.HUMIDITY+4])[0]
                )
            }
            
            return results
            
        except serial.SerialException as e:
            logger.error(f"Error reading registers: {e}")
            return {}
    
    def measure(self) -> Optional[SensorMeasurement]:
        """
        Perform a complete measurement cycle
        
        Returns:
            SensorMeasurement object with the results, or None if measurement failed
        """
        if not self.start_measurement():
            logger.error("Failed to start measurement")
            return None
        
        # Wait for measurement to complete
        time.sleep(0.5)
        
        result = self.read_registers()
        if not result:
            return None
        
        measurement = result.get("measurement")
        if measurement:
            logger.info(f"Measurement: Concentration={measurement.concentration:.2f}, "
                       f"Temperature={measurement.temperature:.1f}°C, "
                       f"Humidity={measurement.humidity:.1f}%")
        
        return measurement
    
    def close(self) -> None:
        """Close the serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("Connection closed")


if __name__ == "__main__":
    # Example usage
    try:
        # Create sensor device interface
        sensor = SensorDevice(port="COM15", baudrate=9600)
        
        # Perform a measurement
        measurement = sensor.measure()
        
        if measurement:
            print(f"Concentration: {measurement.concentration}")
            print(f"Temperature: {measurement.temperature}°C")
            print(f"Humidity: {measurement.humidity}%")
            print(f"Timestamp: {measurement.timestamp}")
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        # Make sure to close the connection
        if 'sensor' in locals():
            sensor.close()