
#include <Arduino.h>
#include <LDS_LDROBOT_LD14P.h>

HardwareSerial lidarSerial(1);
LDS_LDROBOT_LD14P lidar;

int readLidar() {
    return lidarSerial.read();
}

size_t writeLidar(const uint8_t* data, size_t len) {
    return lidarSerial.write(data, len);
}

void onPoint(float angle, float distance, float quality, bool scanCompleted) {
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" | Distance: ");
    Serial.print(distance);
    Serial.print(" mm | Quality: ");
    Serial.println(quality);
}

void setup() {
    Serial.begin(115200);

    lidarSerial.begin(
        lidar.getSerialBaudRate(),
        SERIAL_8N1,
        16,  // RX
        17   // TX
    );

    lidar.setSerialReadCallback(readLidar);
    lidar.setSerialWriteCallback(writeLidar);
    lidar.setScanPointCallback(onPoint);

    lidar.init();

    LDS::result_t result = lidar.start();

    Serial.println(lidar.resultCodeToString(result));
}

void loop() {
    lidar.loop();
}
