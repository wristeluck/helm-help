#include <mmc.h>
#include <file_store.h>

FileStore fileStore(640); // catalog (first) sector of data file

void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600); // speed of GPS NMEA
   fileStore.initialiseCatalog();
    fileStore.logSeparator();

}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long current_time = millis();
  int32_t qw=1000, qx=1, qy=2, qz=3;  // quaternion
  double target_yaw=100.01, current_yaw=99.999, target_rudder=90; // setpoint, input, output

  fileStore.logQuaternion(current_time, (int16_t)qw, (int16_t)qx, (int16_t)qy, (int16_t)qz);
  fileStore.logPid(current_time, target_yaw, current_yaw, target_rudder);

}
