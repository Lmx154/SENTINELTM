#include <Arduino.h> // Explicitly include Arduino core library

// Base Sensor class (interface)
class Sensor {
public:
    virtual String read() = 0; // Pure virtual method to read sensor data
    virtual ~Sensor() {}      // Virtual destructor for proper cleanup
};

// Simulated Barometer (BMI088) - Provides pressure and altitude
class SimulatedBarometer : public Sensor {
public:
    String read() override {
        float pressure = random(95000, 105000) / 100.0; // hPa (950-1050)
        float altitude = random(0, 10000) / 10.0;      // meters (0-1000m)
        return "baro_press:" + String(pressure) + ",altitude:" + String(altitude);
    }
};

// Simulated Magnetometer (IIS2MDCTR) - Magnetic field in x, y, z
class SimulatedMagnetometer : public Sensor {
public:
    String read() override {
        float mag_x = random(-500, 500) / 10.0; // µT
        float mag_y = random(-500, 500) / 10.0;
        float mag_z = random(-500, 500) / 10.0;
        return "mag_x:" + String(mag_x) + ",mag_y:" + String(mag_y) + ",mag_z:" + String(mag_z);
    }
};

// Simulated Pressure Sensor (MPRLS0025PA00001A)
class SimulatedPressureSensor : public Sensor {
public:
    String read() override {
        float pressure = random(95000, 105000) / 100.0; // hPa
        return "press:" + String(pressure);
    }
};

// Simulated GPS Receiver (u-blox MAX-8Q)
class SimulatedGPS : public Sensor {
private:
    uint8_t satellites = 8; // Simulated number of satellites (fixed for simplicity)

public:
    String read() override {
        float lat = 37.7749 + (random(-100, 100) / 10000.0); // Latitude
        float lon = -122.4194 + (random(-100, 100) / 10000.0); // Longitude
        satellites = random(4, 12); // Vary between 4-12 satellites
        return "latitude:" + String(lat, 6) + ",longitude:" + String(lon, 6) + ",satellites:" + String(satellites);
    }
};

// Simulated Temperature Sensor (TMP100)
class SimulatedTempSensor : public Sensor {
public:
    String read() override {
        float temp = random(200, 300) / 10.0; // 20-30°C
        return "temp:" + String(temp);
    }
};

// Simulated BMI088 Accelerometer - Acceleration and velocity
class SimulatedAccelerometer : public Sensor {
public:
    String read() override {
        float acc_x = random(-100, 100) / 10.0; // m/s²
        float acc_y = random(-100, 100) / 10.0;
        float acc_z = random(-100, 100) / 10.0;
        float vel_x = random(-50, 50) / 10.0;   // m/s (velocity)
        float vel_y = random(-50, 50) / 10.0;
        float vel_z = random(-50, 50) / 10.0;
        return "acceleration_x:" + String(acc_x) + ",acceleration_y:" + String(acc_y) + 
               ",acceleration_z:" + String(acc_z) + ",velocity_x:" + String(vel_x) + 
               ",velocity_y:" + String(vel_y) + ",velocity_z:" + String(vel_z);
    }
};

// Simulated BMI088 Gyroscope - Pitch, yaw, roll
class SimulatedGyroscope : public Sensor {
public:
    String read() override {
        float pitch = random(-1000, 1000) / 10.0; // deg/s
        float roll = random(-1000, 1000) / 10.0;
        float yaw = random(-1000, 1000) / 10.0;
        return "pitch:" + String(pitch) + ",roll:" + String(roll) + ",yaw:" + String(yaw);
    }
};

// Base Communication class (interface)
class Communication {
public:
    virtual void send(const String& data) = 0;
    virtual String receive() = 0;
    virtual ~Communication() {}
};

// Simulated LoRa communication (RFM95CW)
class SimulatedLoRa : public Communication {
private:
    String lastSentData;
    int32_t rssi = -50; // Simulated RSSI in dBm

public:
    void send(const String& data) override {
        lastSentData = data;
        rssi = random(-120, -30); // Simulate RSSI variation
    }
    String receive() override {
        String data = lastSentData;
        lastSentData = ""; // Clear after reading
        return data;
    }
    int32_t getRssi() { return rssi; }
};

// Navigation Computer (NAVC) - Simulates sensor data collection
class NAVC {
private:
    Sensor* barometer;
    Sensor* magnetometer;
    Sensor* pressureSensor;
    Sensor* gps;
    Sensor* tempSensor;
    Sensor* accelerometer;
    Sensor* gyroscope;
    uint32_t id = 1; // Simulated pub id

public:
    NAVC() {
        barometer = new SimulatedBarometer();
        magnetometer = new SimulatedMagnetometer();
        pressureSensor = new SimulatedPressureSensor();
        gps = new SimulatedGPS();
        tempSensor = new SimulatedTempSensor();
        accelerometer = new SimulatedAccelerometer();
        gyroscope = new SimulatedGyroscope();
    }

    ~NAVC() {
        delete barometer;
        delete magnetometer;
        delete pressureSensor;
        delete gps;
        delete tempSensor;
        delete accelerometer;
        delete gyroscope;
    }

    String collectData() {
        unsigned long currentTime = millis();
        uint32_t minute = (currentTime / 60000) % 60; // Minutes since start
        uint32_t second = (currentTime / 1000) % 60;  // Seconds since start
        String missionTime = String(minute) + "m" + String(second) + "s";

        String data = "[id:" + String(id) + ",";
        data += "mission_time:" + missionTime + ",";
        data += "connected:1,"; // Simulate always connected
        data += accelerometer->read() + ",";
        data += gyroscope->read() + ",";
        data += magnetometer->read() + ",";
        data += barometer->read() + ",";
        data += pressureSensor->read() + ",";
        data += gps->read() + ",";
        data += tempSensor->read() + ",";
        data += "battery:" + String(random(370, 420) / 100.0) + ","; // Battery 3.7-4.2V
        data += "minute:" + String(minute) + ",";
        data += "second:" + String(second) + "]";
        return data;
    }
};

// Flight Controller (FC) - Manages data transmission
class FC {
private:
    Communication* comm;

public:
    FC(Communication* communication) : comm(communication) {}

    void transmit(const String& data) {
        comm->send(data);
    }

    int32_t getRssi() {
        return static_cast<SimulatedLoRa*>(comm)->getRssi();
    }
};

// Ground Station - Receives data and forwards to UART
class GroundStation {
private:
    Communication* comm;

public:
    GroundStation(Communication* communication) : comm(communication) {}

    void receiveAndForward() {
        String data = comm->receive();
        if (data != "") {
            int32_t rssi = static_cast<SimulatedLoRa*>(comm)->getRssi();
            Serial.println(data + ",rssi:" + String(rssi));
        }
    }
};

// Global instances
NAVC navc;
SimulatedLoRa lora;
FC fc(&lora);
GroundStation groundStation(&lora);

void setup() {
    Serial.begin(115200); // Initialize UART at 115200 baud
    delay(1000);          // Wait for Serial to stabilize
    Serial.println("Rocket Launch Simulation Started");
}

void loop() {
    // Rocket function: NAVC collects data, FC transmits it
    String sensorData = navc.collectData();
    fc.transmit(sensorData);

    // Ground station function: Receive and forward data
    groundStation.receiveAndForward();

    delay(1000); // Simulate 1-second intervals between transmissions
}