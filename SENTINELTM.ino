#include <Arduino.h> // Explicitly include Arduino core library

// Base Sensor class (interface)
class Sensor {
public:
    virtual String read() = 0; // Pure virtual method to read sensor data
    virtual ~Sensor() {}      // Virtual destructor for proper cleanup
};

// Simulated Barometer (BMI088) - Provides pressure and altitude (simplified)
class SimulatedBarometer : public Sensor {
public:
    String read() override {
        float pressure = random(95000, 105000) / 100.0; // hPa (950-1050)
        return "baro_press:" + String(pressure);
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
public:
    String read() override {
        float lat = 37.7749 + (random(-100, 100) / 10000.0); // Simulate small movement
        float lon = -122.4194 + (random(-100, 100) / 10000.0);
        return "gps_lat:" + String(lat, 6) + ",gps_lon:" + String(lon, 6);
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

// Simulated BMI088 Accelerometer
class SimulatedAccelerometer : public Sensor {
public:
    String read() override {
        float acc_x = random(-100, 100) / 10.0; // m/s²
        float acc_y = random(-100, 100) / 10.0;
        float acc_z = random(-100, 100) / 10.0;
        return "acc_x:" + String(acc_x) + ",acc_y:" + String(acc_y) + ",acc_z:" + String(acc_z);
    }
};

// Simulated BMI088 Gyroscope
class SimulatedGyroscope : public Sensor {
public:
    String read() override {
        float gyro_x = random(-1000, 1000) / 10.0; // deg/s (pitch)
        float gyro_y = random(-1000, 1000) / 10.0; // (roll)
        float gyro_z = random(-1000, 1000) / 10.0; // (yaw)
        return "gyro_x:" + String(gyro_x) + ",gyro_y:" + String(gyro_y) + ",gyro_z:" + String(gyro_z);
    }
};

// Base Communication class (interface)
class Communication {
public:
    virtual void send(const String& data) = 0;
    virtual String receive() = 0;
    virtual ~Communication() {}
};

// Simulated LoRa communication
class SimulatedLoRa : public Communication {
private:
    String lastSentData;
public:
    void send(const String& data) override {
        lastSentData = data; // Store data for ground station to "receive"
    }
    String receive() override {
        String data = lastSentData;
        lastSentData = ""; // Clear after reading (simulate one-time receive)
        return data;
    }
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
        String data = "[timestamp:" + String(millis()) + ",";
        data += accelerometer->read() + ",";
        data += gyroscope->read() + ",";
        data += magnetometer->read() + ",";
        data += barometer->read() + ",";
        data += pressureSensor->read() + ",";
        data += gps->read() + ",";
        data += tempSensor->read() + "]";
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
        comm->send(data); // Send data via communication module
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
            Serial.println(data); // Output to UART (Serial monitor)
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