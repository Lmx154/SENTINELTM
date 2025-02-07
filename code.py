import struct
import random
import time

# BasePacket: Common header and timestamp
class BasePacket:
    def __init__(self, packet_type, packet_id, year, month, day, hour, minute, second,
                 sender_id=1, receiver_id=1):
        # Compute header A from sender/receiver.
        # (Here we pack sender in the high nibble and receiver in the low nibble.)
        self.sender_id = sender_id
        self.receiver_id = receiver_id
        self.header_a = self.compute_header_a(sender_id, receiver_id)
        # For these packets the header B is used to hold the packet type.
        self.header_b = packet_type
        self.packet_id = packet_id
        self.year = year
        self.month = month
        self.day = day
        self.hour = hour
        self.minute = minute
        self.second = second

    def compute_header_a(self, sender, receiver):
        # Pack sender (4 bits) into the high nibble and receiver (4 bits) into the low nibble.
        return ((sender & 0x0F) << 4) | (receiver & 0x0F)

    def pack_header(self):
        # Header A: uint8, Header B: uint8, id: uint16 (little-endian)
        return struct.pack("<BBH", self.header_a, self.header_b, self.packet_id)

    def pack_timestamp(self):
        # Timestamp: year (uint16), month, day, hour, minute, second (each uint8)
        return struct.pack("<HBBBBB", self.year, self.month, self.day,
                           self.hour, self.minute, self.second)

    def pack(self):
        # Must be overridden by subclasses to pack additional fields.
        raise NotImplementedError

    def get_packet_bytes(self):
        return self.pack()


# Telemetry A (All Values)
class TelemetryA(BasePacket):
    PACKET_TYPE = 130

    def __init__(self, packet_id, year, month, day, hour, minute, second,
                 accel_x, accel_y, accel_z, vel_x, vel_y, vel_z,
                 pitch, roll, yaw, temperature, pressure, altitude,
                 humidity, latitude, longitude, satellites, rssi, snr, battery,
                 sender_id=1, receiver_id=1):
        super().__init__(TelemetryA.PACKET_TYPE, packet_id, year, month, day,
                         hour, minute, second, sender_id, receiver_id)
        self.accel_x = accel_x   # int16, value *100 (m/s²)
        self.accel_y = accel_y   # int16
        self.accel_z = accel_z   # int16
        self.vel_x = vel_x       # int16, value *10 (m/s)
        self.vel_y = vel_y       # int16
        self.vel_z = vel_z       # int16
        self.pitch = pitch       # int16, value *10 (deg)
        self.roll = roll         # int16
        self.yaw = yaw           # int16
        self.temperature = temperature  # int16, value *10 (°C)
        self.pressure = pressure          # uint16, value/10 (hPa)
        self.altitude = altitude          # int16, in meters
        self.humidity = humidity          # uint16, value *100 (%)
        self.latitude = latitude          # float, in degrees
        self.longitude = longitude        # float, in degrees
        self.satellites = satellites      # uint8, count
        self.rssi = rssi                  # int16, in dBm
        self.snr = snr                    # int16, value *10 (dB)
        self.battery = battery            # uint16, in millivolts

    def pack(self):
        # Build header and timestamp (11 bytes total)
        header = self.pack_header() + self.pack_timestamp()
        # Payload format (all little‐endian):
        # Three int16’s for accel, three for velocity, three for angles,
        # temperature (h), pressure (H), altitude (h), humidity (H),
        # two floats (latitude, longitude), satellites (B),
        # rssi (h), snr (h), battery (H).
        fmt = "<" + "hhh" + "hhh" + "hhh" + "h" + "H" + "h" + "H" + "ff" + "B" + "hh" + "H"
        payload = struct.pack(fmt,
                              self.accel_x, self.accel_y, self.accel_z,
                              self.vel_x, self.vel_y, self.vel_z,
                              self.pitch, self.roll, self.yaw,
                              self.temperature, self.pressure,
                              self.altitude, self.humidity,
                              self.latitude, self.longitude,
                              self.satellites, self.rssi, self.snr,
                              self.battery)
        return header + payload

    @classmethod
    def random_packet(cls):
        now = time.localtime()  # returns (year, month, day, hour, minute, second, …)
        packet_id = random.randint(0, 65535)
        year = now[0]
        month = now[1]
        day = now[2]
        hour = now[3]
        minute = now[4]
        second = now[5]
        # Random int16 values for acceleration, velocity, etc.
        accel_x = random.randint(-32768, 32767)
        accel_y = random.randint(-32768, 32767)
        accel_z = random.randint(-32768, 32767)
        vel_x = random.randint(-32768, 32767)
        vel_y = random.randint(-32768, 32767)
        vel_z = random.randint(-32768, 32767)
        pitch = random.randint(-32768, 32767)
        roll = random.randint(-32768, 32767)
        yaw = random.randint(-32768, 32767)
        temperature = random.randint(-32768, 32767)
        pressure = random.randint(0, 65535)
        altitude = random.randint(-32768, 32767)
        humidity = random.randint(0, 65535)
        latitude = random.uniform(-90.0, 90.0)
        longitude = random.uniform(-180.0, 180.0)
        satellites = random.randint(0, 255)
        rssi = random.randint(-32768, 32767)
        snr = random.randint(-32768, 32767)
        battery = random.randint(0, 65535)
        return cls(packet_id, year, month, day, hour, minute, second,
                   accel_x, accel_y, accel_z,
                   vel_x, vel_y, vel_z,
                   pitch, roll, yaw,
                   temperature, pressure, altitude,
                   humidity, latitude, longitude,
                   satellites, rssi, snr, battery)


# Telemetry B (MPU Values)
class TelemetryB(BasePacket):
    PACKET_TYPE = 131

    def __init__(self, packet_id, year, month, day, hour, minute, second,
                 accel_x, accel_y, accel_z, vel_x, vel_y, vel_z,
                 pitch, roll, yaw, rssi, snr,
                 sender_id=1, receiver_id=1):
        super().__init__(TelemetryB.PACKET_TYPE, packet_id, year, month, day,
                         hour, minute, second, sender_id, receiver_id)
        self.accel_x = accel_x
        self.accel_y = accel_y
        self.accel_z = accel_z
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.vel_z = vel_z
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        self.rssi = rssi
        self.snr = snr

    def pack(self):
        header = self.pack_header() + self.pack_timestamp()
        # Payload: 3x accel (hhh), 3x velocity (hhh), 3x angles (hhh), then rssi and snr (hh)
        fmt = "<" + "hhh" + "hhh" + "hhh" + "hh"
        payload = struct.pack(fmt,
                              self.accel_x, self.accel_y, self.accel_z,
                              self.vel_x, self.vel_y, self.vel_z,
                              self.pitch, self.roll, self.yaw,
                              self.rssi, self.snr)
        return header + payload

    @classmethod
    def random_packet(cls):
        now = time.localtime()
        packet_id = random.randint(0, 65535)
        year, month, day, hour, minute, second = now[0:6]
        accel_x = random.randint(-32768, 32767)
        accel_y = random.randint(-32768, 32767)
        accel_z = random.randint(-32768, 32767)
        vel_x = random.randint(-32768, 32767)
        vel_y = random.randint(-32768, 32767)
        vel_z = random.randint(-32768, 32767)
        pitch = random.randint(-32768, 32767)
        roll = random.randint(-32768, 32767)
        yaw = random.randint(-32768, 32767)
        rssi = random.randint(-32768, 32767)
        snr = random.randint(-32768, 32767)
        return cls(packet_id, year, month, day, hour, minute, second,
                   accel_x, accel_y, accel_z,
                   vel_x, vel_y, vel_z,
                   pitch, roll, yaw,
                   rssi, snr)


# Telemetry C (BME Values)
class TelemetryC(BasePacket):
    PACKET_TYPE = 132

    def __init__(self, packet_id, year, month, day, hour, minute, second,
                 temperature, pressure, altitude, humidity, rssi, snr,
                 sender_id=1, receiver_id=1):
        super().__init__(TelemetryC.PACKET_TYPE, packet_id, year, month, day,
                         hour, minute, second, sender_id, receiver_id)
        self.temperature = temperature
        self.pressure = pressure
        self.altitude = altitude
        self.humidity = humidity
        self.rssi = rssi
        self.snr = snr

    def pack(self):
        header = self.pack_header() + self.pack_timestamp()
        # Payload: temperature (h), pressure (H), altitude (h), humidity (H), rssi (h), snr (h)
        fmt = "<hHhHhh"
        payload = struct.pack(fmt,
                              self.temperature, self.pressure,
                              self.altitude, self.humidity,
                              self.rssi, self.snr)
        return header + payload

    @classmethod
    def random_packet(cls):
        now = time.localtime()
        packet_id = random.randint(0, 65535)
        year, month, day, hour, minute, second = now[0:6]
        temperature = random.randint(-32768, 32767)
        pressure = random.randint(0, 65535)
        altitude = random.randint(-32768, 32767)
        humidity = random.randint(0, 65535)
        rssi = random.randint(-32768, 32767)
        snr = random.randint(-32768, 32767)
        return cls(packet_id, year, month, day, hour, minute, second,
                   temperature, pressure, altitude, humidity, rssi, snr)


# Telemetry D (GPS Values)
class TelemetryD(BasePacket):
    PACKET_TYPE = 133

    def __init__(self, packet_id, year, month, day, hour, minute, second,
                 latitude, longitude, satellites, rssi, snr,
                 sender_id=1, receiver_id=1):
        super().__init__(TelemetryD.PACKET_TYPE, packet_id, year, month, day,
                         hour, minute, second, sender_id, receiver_id)
        self.latitude = latitude
        self.longitude = longitude
        self.satellites = satellites
        self.rssi = rssi
        self.snr = snr

    def pack(self):
        header = self.pack_header() + self.pack_timestamp()
        # Payload: latitude (f), longitude (f), satellites (B), rssi (h), snr (h)
        fmt = "<ffBhh"
        payload = struct.pack(fmt,
                              self.latitude, self.longitude,
                              self.satellites, self.rssi, self.snr)
        return header + payload

    @classmethod
    def random_packet(cls):
        now = time.localtime()
        packet_id = random.randint(0, 65535)
        year, month, day, hour, minute, second = now[0:6]
        latitude = random.uniform(-90.0, 90.0)
        longitude = random.uniform(-180.0, 180.0)
        satellites = random.randint(0, 255)
        rssi = random.randint(-32768, 32767)
        snr = random.randint(-32768, 32767)
        return cls(packet_id, year, month, day, hour, minute, second,
                   latitude, longitude, satellites, rssi, snr)


# Function to randomly choose one packet type and "emit" it.
def emit_random_packet():
    # List of packet classes to choose from
    packet_classes = [TelemetryA, TelemetryB, TelemetryC, TelemetryD]
    chosen_class = random.choice(packet_classes)
    packet = chosen_class.random_packet()
    packet_bytes = packet.get_packet_bytes()
    # In an actual application, you might send packet_bytes over a serial port.
    # For demonstration we print its type and hexadecimal representation.
    print("Emitting packet type {} ({} bytes):".format(chosen_class.__name__,
                                                       len(packet_bytes)))
    print(packet_bytes.hex())


# Main: emit one random packet 4 times per second indefinitely.
if __name__ == '__main__':
    while True:
        emit_random_packet()
        time.sleep(0.25)  # Wait 250 ms (i.e. 4 packets per second)

