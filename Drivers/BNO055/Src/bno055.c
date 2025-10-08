#include "bno055.h"

#include <string.h>

uint16_t accelScale = 100;
uint16_t tempScale = 1;
uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
uint16_t magScale = 16;
uint16_t quaScale = (1 << 14);  // 2^14

bool bno055_setPage(uint8_t page) {
  return bno055_writeData(BNO055_PAGE_ID, page);
}

bool bno055_getOperationMode(bno055_opmode_t *mode) {
  return bno055_readData(BNO055_OPR_MODE, (uint8_t *)mode, 1);
}

bno055_opmode_t bno055_getOperationModeWithoutCheck() {
  bno055_opmode_t mode;
  bno055_readData(BNO055_OPR_MODE, (uint8_t *)&mode, 1);
  return mode;
}

bool bno055_setOperationMode(bno055_opmode_t mode) {
  bool result = bno055_writeData(BNO055_OPR_MODE, mode);
  if (mode == BNO055_OPERATION_MODE_CONFIG) {
    bno055_delay(50);
  } else {
    bno055_delay(20);
  }
  return result;
}

bool bno055_setOperationModeConfig() {
  return bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

bool bno055_setOperationModeNDOF() {
  return bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}

bool bno055_setExternalCrystalUse(bool state) {
  if (!bno055_setPage(0)) { return false; }

  uint8_t tmp = 0;
  if (!bno055_readData(BNO055_SYS_TRIGGER, &tmp, 1)) { return false; }

  tmp |= (state == true) ? 0x80 : 0x0;
  bool result = bno055_writeData(BNO055_SYS_TRIGGER, tmp);
  bno055_delay(1000);
  return result;
}

bool bno055_enableExternalCrystal() {
  return bno055_setExternalCrystalUse(true);
}

bool bno055_disableExternalCrystal() {
  return bno055_setExternalCrystalUse(false);
}

bool bno055_reset() {
  bool result = bno055_writeData(BNO055_SYS_TRIGGER, 0x20);
  bno055_delay(1000);
  return result;
}

bool bno055_getTemp(int8_t *temp) {
  if (!bno055_setPage(0)) { return false; }

  uint8_t t;
  if (!bno055_readData(BNO055_TEMP, &t, 1)) { return false; }

  *temp = (int8_t)t;
  return true;
}

int8_t bno055_getTempWithoutCheck() {
  bno055_setPage(0);
  uint8_t t;
  bno055_readData(BNO055_TEMP, &t, 1);
  return (int8_t)t;
}

bool bno055_setup() {
  if (!bno055_reset()) { return false; }

  uint8_t id = 0;
  if (!bno055_readData(BNO055_CHIP_ID, &id, 1)) { return false; }

  if (id != BNO055_ID) {
    printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
    return false;
  }

  if (!bno055_setPage(0)) { return false; }

  if (!bno055_writeData(BNO055_SYS_TRIGGER, 0x0)) { return false; }

  // Select BNO055 config mode
  if (!bno055_setOperationModeConfig()) { return false; }

  bno055_delay(25);

  return true;
}

bool bno055_getSWRevision(int16_t *swRev) {
  if (!bno055_setPage(0)) { return false; }

  uint8_t buffer[2];
  if (!bno055_readData(BNO055_SW_REV_ID_LSB, buffer, 2)) { return false; }

  *swRev = (int16_t)((buffer[1] << 8) | buffer[0]);
  return true;
}

int16_t bno055_getSWRevisionWithoutCheck() {
  bno055_setPage(0);
  uint8_t buffer[2];
  bno055_readData(BNO055_SW_REV_ID_LSB, buffer, 2);
  return (int16_t)((buffer[1] << 8) | buffer[0]);
}

bool bno055_getBootloaderRevision(uint8_t *blRev) {
  if (!bno055_setPage(0)) { return false; }

  return bno055_readData(BNO055_BL_REV_ID, blRev, 1);
}

uint8_t bno055_getBootloaderRevisionWithoutCheck() {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_BL_REV_ID, &tmp, 1);
  return tmp;
}

bool bno055_getSystemStatus(uint8_t *status) {
  if (!bno055_setPage(0)) { return false; }

  return bno055_readData(BNO055_SYS_STATUS, status, 1);
}

uint8_t bno055_getSystemStatusWithoutCheck() {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_STATUS, &tmp, 1);
  return tmp;
}

bool bno055_getSelfTestResult(bno055_self_test_result_t *result) {
  if (!bno055_setPage(0)) { return false; }

  uint8_t tmp;
  if (!bno055_readData(BNO055_ST_RESULT, &tmp, 1)) { return false; }

  result->mcuState = (tmp >> 3) & 0x01;
  result->gyrState = (tmp >> 2) & 0x01;
  result->magState = (tmp >> 1) & 0x01;
  result->accState = (tmp >> 0) & 0x01;
  return true;
}

bno055_self_test_result_t bno055_getSelfTestResultWithoutCheck() {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_self_test_result_t res = {.mcuState = 0, .gyrState = 0, .magState = 0, .accState = 0};
  bno055_readData(BNO055_ST_RESULT, &tmp, 1);
  res.mcuState = (tmp >> 3) & 0x01;
  res.gyrState = (tmp >> 2) & 0x01;
  res.magState = (tmp >> 1) & 0x01;
  res.accState = (tmp >> 0) & 0x01;
  return res;
}

bool bno055_getSystemError(uint8_t *error) {
  if (!bno055_setPage(0)) { return false; }

  return bno055_readData(BNO055_SYS_ERR, error, 1);
}

uint8_t bno055_getSystemErrorWithoutCheck() {
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_ERR, &tmp, 1);
  return tmp;
}

bool bno055_getCalibrationState(bno055_calibration_state_t *cal) {
  if (!bno055_setPage(0)) { return false; }

  uint8_t calState = 0;
  if (!bno055_readData(BNO055_CALIB_STAT, &calState, 1)) { return false; }

  cal->sys = (calState >> 6) & 0x03;
  cal->gyro = (calState >> 4) & 0x03;
  cal->accel = (calState >> 2) & 0x03;
  cal->mag = calState & 0x03;
  return true;
}

bno055_calibration_state_t bno055_getCalibrationStateWithoutCheck() {
  bno055_setPage(0);
  bno055_calibration_state_t cal = {.sys = 0, .gyro = 0, .mag = 0, .accel = 0};
  uint8_t calState = 0;
  bno055_readData(BNO055_CALIB_STAT, &calState, 1);
  cal.sys = (calState >> 6) & 0x03;
  cal.gyro = (calState >> 4) & 0x03;
  cal.accel = (calState >> 2) & 0x03;
  cal.mag = calState & 0x03;
  return cal;
}

bool bno055_getCalibrationData(bno055_calibration_data_t *calData) {
  uint8_t buffer[22];
  bno055_opmode_t operationMode;

  if (!bno055_getOperationMode(&operationMode)) { return false; }

  if (!bno055_setOperationModeConfig()) { return false; }

  if (!bno055_setPage(0)) { return false; }

  if (!bno055_readData(BNO055_ACC_OFFSET_X_LSB, buffer, 22)) {
    // Try to restore previous operation mode
    bno055_setOperationMode(operationMode);
    return false;
  }

  // Assumes little endian processor
  memcpy(&calData->offset.accel, buffer, 6);
  memcpy(&calData->offset.mag, buffer + 6, 6);
  memcpy(&calData->offset.gyro, buffer + 12, 6);
  memcpy(&calData->radius.accel, buffer + 18, 2);
  memcpy(&calData->radius.mag, buffer + 20, 2);

  return bno055_setOperationMode(operationMode);
}

bno055_calibration_data_t bno055_getCalibrationDataWithoutCheck() {
  bno055_calibration_data_t calData;
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationModeWithoutCheck();
  bno055_setOperationModeConfig();
  bno055_setPage(0);

  bno055_readData(BNO055_ACC_OFFSET_X_LSB, buffer, 22);

  // Assumes little endian processor
  memcpy(&calData.offset.accel, buffer, 6);
  memcpy(&calData.offset.mag, buffer + 6, 6);
  memcpy(&calData.offset.gyro, buffer + 12, 6);
  memcpy(&calData.radius.accel, buffer + 18, 2);
  memcpy(&calData.radius.mag, buffer + 20, 2);

  bno055_setOperationMode(operationMode);

  return calData;
}

bool bno055_setCalibrationData(bno055_calibration_data_t calData) {
  uint8_t buffer[22];
  bno055_opmode_t operationMode;

  if (!bno055_getOperationMode(&operationMode)) { return false; }

  if (!bno055_setOperationModeConfig()) { return false; }

  if (!bno055_setPage(0)) { return false; }

  // Assumes little endian processor
  memcpy(buffer, &calData.offset.accel, 6);
  memcpy(buffer + 6, &calData.offset.mag, 6);
  memcpy(buffer + 12, &calData.offset.gyro, 6);
  memcpy(buffer + 18, &calData.radius.accel, 2);
  memcpy(buffer + 20, &calData.radius.mag, 2);

  bool success = true;
  for (uint8_t i = 0; i < 22; i++) {
    // TODO(oliv4945): create multibytes write
    if (!bno055_writeData(BNO055_ACC_OFFSET_X_LSB + i, buffer[i])) {
      success = false;
      break;
    }
  }

  // Try to restore previous operation mode even if there was an error
  return bno055_setOperationMode(operationMode) && success;
}

bool bno055_getVector(uint8_t vec, bno055_vector_t *xyz) {
  if (!bno055_setPage(0)) { return false; }

  uint8_t buffer[8];  // Quaternion need 8 bytes
  bool result;

  if (vec == BNO055_VECTOR_QUATERNION) {
    result = bno055_readData(vec, buffer, 8);
  } else {
    result = bno055_readData(vec, buffer, 6);
  }

  if (!result) { return false; }

  double scale = 1;

  if (vec == BNO055_VECTOR_MAGNETOMETER) {
    scale = magScale;
  } else if (vec == BNO055_VECTOR_ACCELEROMETER || vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GRAVITY) {
    scale = accelScale;
  } else if (vec == BNO055_VECTOR_GYROSCOPE) {
    scale = angularRateScale;
  } else if (vec == BNO055_VECTOR_EULER) {
    scale = eulerScale;
  } else if (vec == BNO055_VECTOR_QUATERNION) {
    scale = quaScale;
  }

  xyz->w = 0;
  if (vec == BNO055_VECTOR_QUATERNION) {
    xyz->w = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz->x = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz->y = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
    xyz->z = (int16_t)((buffer[7] << 8) | buffer[6]) / scale;
  } else {
    xyz->x = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz->y = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz->z = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
  }

  return true;
}

bno055_vector_t bno055_getVectorWithoutCheck(uint8_t vec) {
  bno055_setPage(0);
  uint8_t buffer[8];  // Quaternion need 8 bytes

  if (vec == BNO055_VECTOR_QUATERNION)
    bno055_readData(vec, buffer, 8);
  else
    bno055_readData(vec, buffer, 6);

  double scale = 1;

  if (vec == BNO055_VECTOR_MAGNETOMETER) {
    scale = magScale;
  } else if (vec == BNO055_VECTOR_ACCELEROMETER || vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GRAVITY) {
    scale = accelScale;
  } else if (vec == BNO055_VECTOR_GYROSCOPE) {
    scale = angularRateScale;
  } else if (vec == BNO055_VECTOR_EULER) {
    scale = eulerScale;
  } else if (vec == BNO055_VECTOR_QUATERNION) {
    scale = quaScale;
  }

  bno055_vector_t xyz = {.w = 0, .x = 0, .y = 0, .z = 0};
  if (vec == BNO055_VECTOR_QUATERNION) {
    xyz.w = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.x = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.y = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
    xyz.z = (int16_t)((buffer[7] << 8) | buffer[6]) / scale;
  } else {
    xyz.x = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.y = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.z = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
  }

  return xyz;
}

bool bno055_getVectorAccelerometer(bno055_vector_t *vec) {
  return bno055_getVector(BNO055_VECTOR_ACCELEROMETER, vec);
}

bool bno055_getVectorMagnetometer(bno055_vector_t *vec) {
  return bno055_getVector(BNO055_VECTOR_MAGNETOMETER, vec);
}

bool bno055_getVectorGyroscope(bno055_vector_t *vec) {
  return bno055_getVector(BNO055_VECTOR_GYROSCOPE, vec);
}

bool bno055_getVectorEuler(bno055_vector_t *vec) {
  return bno055_getVector(BNO055_VECTOR_EULER, vec);
}

bool bno055_getVectorLinearAccel(bno055_vector_t *vec) {
  return bno055_getVector(BNO055_VECTOR_LINEARACCEL, vec);
}

bool bno055_getVectorGravity(bno055_vector_t *vec) {
  return bno055_getVector(BNO055_VECTOR_GRAVITY, vec);
}

bool bno055_getVectorQuaternion(bno055_vector_t *vec) {
  return bno055_getVector(BNO055_VECTOR_QUATERNION, vec);
}

// Legacy functions that don't check return values
bno055_vector_t bno055_getVectorAccelerometerWithoutCheck() {
  return bno055_getVectorWithoutCheck(BNO055_VECTOR_ACCELEROMETER);
}

bno055_vector_t bno055_getVectorMagnetometerWithoutCheck() {
  return bno055_getVectorWithoutCheck(BNO055_VECTOR_MAGNETOMETER);
}

bno055_vector_t bno055_getVectorGyroscopeWithoutCheck() {
  return bno055_getVectorWithoutCheck(BNO055_VECTOR_GYROSCOPE);
}

bno055_vector_t bno055_getVectorEulerWithoutCheck() {
  return bno055_getVectorWithoutCheck(BNO055_VECTOR_EULER);
}

bno055_vector_t bno055_getVectorLinearAccelWithoutCheck() {
  return bno055_getVectorWithoutCheck(BNO055_VECTOR_LINEARACCEL);
}

bno055_vector_t bno055_getVectorGravityWithoutCheck() {
  return bno055_getVectorWithoutCheck(BNO055_VECTOR_GRAVITY);
}

bno055_vector_t bno055_getVectorQuaternionWithoutCheck() {
  return bno055_getVectorWithoutCheck(BNO055_VECTOR_QUATERNION);
}

bool bno055_setAxisMap(bno055_axis_map_t axis) {
  uint8_t axisRemap = (axis.z << 4) | (axis.y << 2) | (axis.x);
  uint8_t axisMapSign = (axis.x_sign << 2) | (axis.y_sign << 1) | (axis.z_sign);

  if (!bno055_writeData(BNO055_AXIS_MAP_CONFIG, axisRemap)) { return false; }

  return bno055_writeData(BNO055_AXIS_MAP_SIGN, axisMapSign);
}
