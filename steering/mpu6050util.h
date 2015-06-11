#include <MPU6050.h>

//#define DEBUG
#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
#endif


uint8_t dmpInitialize(MPU6050 mpu, FileStore fileStore) {
    // reset device
    mpu.reset();
    delay(30); // wait after reset

    // enable sleep mode and wake cycle
    /*Serial.println(F("Enabling sleep mode..."));
    setSleepEnabled(true);
    Serial.println(F("Enabling wake cycle..."));
    setWakeCycleEnabled(true);*/

    // disable sleep mode
    mpu.setSleepEnabled(false);

    // get MPU hardware revision
    mpu.setMemoryBank(0x10, true, true);
    mpu.setMemoryStartAddress(0x06);
    uint8_t hwRevision = mpu.readMemoryByte();
    DEBUG_PRINT(F("Revision @ user[16][6] = "));
    DEBUG_PRINTLNF(hwRevision, HEX);
    mpu.setMemoryBank(0, false, false);

    // check OTP bank valid
    uint8_t otpValid = mpu.getOTPBankValid();
    DEBUG_PRINT(F("OTP bank is "));
    DEBUG_PRINTLN(otpValid ? F("valid!") : F("invalid!"));

    // get X/Y/Z gyro offsets
    int8_t xgOffsetTC = mpu.getXGyroOffsetTC();
    int8_t ygOffsetTC = mpu.getYGyroOffsetTC();
    int8_t zgOffsetTC = mpu.getZGyroOffsetTC();
    DEBUG_PRINT(F("X gyro offset = "));
    DEBUG_PRINTLN(xgOffsetTC);
    DEBUG_PRINT(F("Y gyro offset = "));
    DEBUG_PRINTLN(ygOffsetTC);
    DEBUG_PRINT(F("Z gyro offset = "));
    DEBUG_PRINTLN(zgOffsetTC);

    // setup weird slave stuff (?)
    mpu.setSlaveAddress(0, 0x7F);
    mpu.setI2CMasterModeEnabled(false);
    mpu.setSlaveAddress(0, 0x68);
    mpu.resetI2CMaster();
    delay(20);

    // load DMP code into memory banks
    uint8_t *memblock;
    bool success;
    memblock = fileStore.getMpuData(0);
    success = mpu.writeDMPConfigurationSet(memblock, 258);
    if(success) Serial.println ("0 S"); else Serial.println("0 F");
    memblock = fileStore.getMpuData(1);
    success = mpu.writeDMPConfigurationSet(memblock, 258);
    if(success) Serial.println ("1 S"); else Serial.println("1 F");
    memblock = fileStore.getMpuData(2);
    success = mpu.writeDMPConfigurationSet(memblock, 258);
    if(success) Serial.println ("2 S"); else Serial.println("2 F");
    memblock = fileStore.getMpuData(3);
    success = mpu.writeDMPConfigurationSet(memblock, 258);
    if(success) Serial.println ("3 S"); else Serial.println("3 F");
    memblock = fileStore.getMpuData(4);
    success = mpu.writeDMPConfigurationSet(memblock, 258);
    if(success) Serial.println ("4 S"); else Serial.println("4 F");
    memblock = fileStore.getMpuData(5);
    success = mpu.writeDMPConfigurationSet(memblock, 258);
    if(success) Serial.println ("5 S"); else Serial.println("5 F");
    memblock = fileStore.getMpuData(6);
    success = mpu.writeDMPConfigurationSet(memblock, 258);
    if(success) Serial.println ("6 S"); else Serial.println("6 F");
    memblock = fileStore.getMpuData(7);
    success = mpu.writeDMPConfigurationSet(memblock, 140);
    if(success) Serial.println ("7 S"); else Serial.println("7 F");
    if (true) {

        // write DMP configuration
        
        memblock = fileStore.getMpuData(8);
        if (mpu.writeDMPConfigurationSet(memblock, 192)) {
            mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
            mpu.setIntEnabled(0x12);
            mpu.setRate(4); // 1khz / (1 + 4) = 200 Hz
            mpu.setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);
            mpu.setDLPFMode(MPU6050_DLPF_BW_42);
            mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
            mpu.setDMPConfig1(0x03);
            mpu.setDMPConfig2(0x00);
            mpu.setOTPBankValid(false);
            mpu.setXGyroOffsetTC(xgOffsetTC);
            mpu.setYGyroOffsetTC(ygOffsetTC);
            mpu.setZGyroOffsetTC(zgOffsetTC);

            //DEBUG_PRINTLN(F("Setting X/Y/Z gyro user offsets to zero..."));
            //setXGyroOffset(0);
            //setYGyroOffset(0);
            //setZGyroOffset(0);

            uint8_t *dmpUpdates = fileStore.getMpuData(9);
//            DEBUG_PRINTLN(F("Writing final memory update 1/7 (function unknown)..."));
            uint8_t dmpUpdate[16], j;
            uint16_t pos = 0;
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu.writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

//            DEBUG_PRINTLN(F("Writing final memory update 2/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu.writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            mpu.resetFIFO();

            uint16_t fifoCount = mpu.getFIFOCount();
            uint8_t fifoBuffer[128];
            mpu.getFIFOBytes(fifoBuffer, fifoCount);
            mpu.setMotionDetectionThreshold(2);
            mpu.setZeroMotionDetectionThreshold(156);
            mpu.setMotionDetectionDuration(80);
            mpu.setZeroMotionDetectionDuration(0);
            mpu.resetFIFO();
            mpu.setFIFOEnabled(true);
            mpu.setDMPEnabled(true);
            mpu.resetDMP();

//            DEBUG_PRINTLN(F("Writing final memory update 3/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu.writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

//            DEBUG_PRINTLN(F("Writing final memory update 4/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu.writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

//            DEBUG_PRINTLN(F("Writing final memory update 5/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu.writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

//            DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
            while ((fifoCount = mpu.getFIFOCount()) < 3);

            mpu.getFIFOBytes(fifoBuffer, fifoCount);

            uint8_t mpuIntStatus = mpu.getIntStatus();

//            DEBUG_PRINTLN(F("Reading final memory update 6/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu.readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            while ((fifoCount = mpu.getFIFOCount()) < 3);

            mpu.getFIFOBytes(fifoBuffer, fifoCount);

            mpuIntStatus = mpu.getIntStatus();

//            DEBUG_PRINTLN(F("Writing final memory update 7/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            mpu.writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            mpu.setDMPEnabled(false);

//            mpu.dmpPacketSize = 42;
            mpu.resetFIFO();
            mpu.getIntStatus();
        } else {
            return 2; // configuration block loading failed
        }
    } else {
        return 1; // main binary block loading failed
    }
    return 0; // success
}

