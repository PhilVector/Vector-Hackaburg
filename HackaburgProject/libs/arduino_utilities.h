/*
 * arduino_utilities.h
 *
 *  Created on: 02.02.2018
 *      Author: WagPhi
 */

#ifndef SRC_OTHERFILES_ARDUINO_UTILITIES_H_
#define SRC_OTHERFILES_ARDUINO_UTILITIES_H_
#ifndef MATLAB_MEX_FILE
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <libudev.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <inttypes.h>
#include <assert.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"

#define TIMEOUT_MICROS 2000000
#define CHAR_PACKET_BUF 100
#define ARD_FRONT 1
#define ARD_REAR_US 3
#define ARD_REAR_IMU 2
#define ARD_ACTOR 4
#define ARD_REMOTE 5
#define ARD_CENTER 6
#define SIZEACTOR 11
#define SENDREQUEST 5

/*! A macro that defines start byte. */
#define START_BYTE      0x02    //STX

/*! A macro that defines end byte. */
#define END_BYTE        0x03    //ETX

/*! A macro that defines escape byte. */
#define ESCAPE_BYTE     0x1B    //ESCAPE

/*! A define for maximum trys to read a specific byte which is received by the arduino. */
#define MAX_MSG_LENGTH_TILL_TIMEOUT 64 // uint8_t

/*!  The maximum size of the std::vector implementation as implemented in utils_arduino.h */
#define MAX_VECTOR_BUFFER_SIZE 128

/*!
 * The Frame header definition of the protocol.
 *
 */

enum ARDUINO_ID /*: uint8_t*/
{
  ARDUINO_NO_NAME = 0,
  ARDUINO_FRONT_US = 1,
  ARDUINO_FRONT_IMU = 2,
  ARDUINO_CENTER_MEASUREMENT = 3,
  ARDUINO_CENTER_ACTUATORS = 4,
  ARDUINO_REAR_US = 5,
  ARDUINO_REAR_IMU_WHEELENC = 6,
  ARDUINO_REMOTE = 7,
};

typedef struct Header
{
  /*! The header id */
  uint8_t ui8ID;
  /*! Length of the data frame */
  uint8_t ui8DataLength;
  /*! The arduino timestamp */
  uint32_t ui32Timestamp;
} tArduinoHeader;

/*!
 * All the sensor ids, which are set in tArduinoHeader.ui8ID.
 *
 * To construct a frame with the specific id, create a tArduinoHeader struct and set the ui8ID
 * to that specific enum. This enums are used to send sensor information to a processing host.
 * The arduino system never processes these ids.
 */
enum SENSOR_ID /*: uint8_t*/{

  ID_ARD_SENSOR_INFO,
  ID_ARD_SENS_ERROR,

  ID_ARD_SENS_US_FRONT_LEFT,
  ID_ARD_SENS_US_FRONT_CENTER_LEFT,
  ID_ARD_SENS_US_FRONT_CENTER,
  ID_ARD_SENS_US_FRONT_CENTER_RIGHT,
  ID_ARD_SENS_US_FRONT_RIGHT,

  ID_ARD_SENS_US_REAR_RIGHT,
  ID_ARD_SENS_US_REAR_CENTER_RIGHT,
  ID_ARD_SENS_US_REAR_CENTER,
  ID_ARD_SENS_US_REAR_CENTER_LEFT,
  ID_ARD_SENS_US_REAR_LEFT,

  ID_ARD_SENS_US_SIDE_LEFT,
  ID_ARD_SENS_US_SIDE_RIGHT,

  ID_ARD_SENS_WHEEL_RIGHT,
  ID_ARD_SENS_WHEEL_LEFT,

  ID_ARD_SENS_IMU,

  ID_ARD_SENS_VOLT_ACTUATOR,
  ID_ARD_SENS_VOLT_ACTUATOR_CELL1,
  ID_ARD_SENS_VOLT_ACTUATOR_CELL2,

  ID_ARD_SENS_VOLT_SENSORS,
  ID_ARD_SENS_VOLT_SENSORS_CELL1,
  ID_ARD_SENS_VOLT_SENSORS_CELL2,
  ID_ARD_SENS_VOLT_SENSORS_CELL3,
  ID_ARD_SENS_VOLT_SENSORS_CELL4,
  ID_ARD_SENS_VOLT_SENSORS_CELL5,
  ID_ARD_SENS_VOLT_SENSORS_CELL6,

  ID_ARD_SENS_REMOTE,
};

typedef struct remotedata
{
  /*! The distance in cm */
  uint16_t ui16Speed;
  uint16_t ui16Steer;
} tRemoteData;


/*!
 * The Info struct.
 *
 * This struct is used as payload (data part) in the protocol. See top of file to see its
 * position.
 */
typedef struct InfoData
{
  /*! see enum ARDUINO_ADDRESS for number(id) of the arduinos */
  uint8_t ui8ArduinoAddress;
  /*! version of the arduino software */
  uint16_t ui16ArduinoVersion;
} tInfoData;

/*!
 * The Error struct.
 *
 * This struct is used as payload (data part) in the protocol. See top of file to see its
 * position. The ui8ErrorNr is filled with an enum (ARD_ERROR) as described below.
 */
typedef struct ErrorData
{
  /*! The error nr */
  int8_t ui8ErrorNr;
}tErrorData;

/*!
 * The Us data struct.
 *
 * This struct is used as payload (data part) in the protocol. See top of file to see its
 * position. Its member i16Distance is filled with the distance and its unit is [cm].
 */
typedef struct UsData
{
  /*! The distance in cm */
  uint16_t ui16Distance;
}tUsData;

/*!
 * The wheel data struct.
 *
 * This struct is used as payload (data part) in the protocol. See top of file to see its
 * position. Its two members are used to indicate the tachometer and the direction of the driving
 * wheels. ui32WheelTach is the tick count of the wheels.
 */
typedef struct SensWheelData
{
  /*! The wheel tach */
  uint32_t ui32WheelTach;
  /*! The wheel direction */
  int8_t i8WheelDir;
}tSensWheelData;

/*!
 * The imu data struct.
 *
 * This struct is used as payload (data part) in the protocol. See top of file to see its
 * position. The units are ...acceleration [g], gyroscope [°/s] and for magnetometer [mG].
 */
typedef struct ImuData
{
  /*! The ax */
  float f32ax;
  /*! The ay */
  float f32ay;
  /*! The az */
  float f32az;

  /*! The gx */
  float f32gx;
  /*! The gy */
  float f32gy;
  /*! The gz */
  float f32gz;

  /*! The mx */
  float f32mx;
  /*! The my */
  float f32my;
  /*! The mz */
  float f32mz;

  /*! The roll */
  float f32roll;
  /*! The pitch */
  float f32pitch;
  /*! The yaw */
  float f32yaw;
}tImuData;

/*!
 * The voltage data struct.
 *
 * This struct is used as payload (data part) in the protocol. See top of file to see its
 * position. The unit is in milliVolt [mV].
 */
typedef struct VoltageData
{
  /*! Information describing the 16 voltage */
  uint16_t ui16VoltageData;
}tVoltageData;

/*!
 * The data union for the protocol frame.
 *
 * This union is a helper define for easier parsing by the receiver. The received frame could be
 * just copied into this union for easy access.
 */
typedef union DataUnion
{
  /*! The information */
  tInfoData info;
  /*! The error */
  tErrorData error;
  /*! The us distance */
  tUsData us;
  /*! The imu data*/
  tImuData imu;
  /*! The wheel data */
  tSensWheelData wheel;
  /*! The voltage data*/
  tVoltageData voltage;
}tDataUnion;

/*!
 * The error codes as used by the ID_ARD_SENS_ERROR id.
 *
 * The elements of this enums are used to fill the member of the tErrorData. The member
 * ui8ErrorNr should be set to one of this enum.
 */
enum ARD_ERROR_CODE /*: int8_t*/
{
  ERROR_STX_NOT_FOUND = -1,
  ERROR_ETX_NOT_FOUND = -2,
  ERROR_ESC_BYTE_BROKEN = -3,
  ERROR_NO_BYTES_AVAILABLE = -4,
  ERROR_CRC_INVALID = -5,
  ERROR_NO_FRAME_DATA = -6,
  ERROR_FRAME_DROPPED = -7,
  ERROR_REMOTE_DETECTED = -8,
  ERROR_NO_GYRO_DETECTED = -9,
  ERROR_INVALID_ACTUATOR_HEADER = -10,
  ERROR_INITIALIZATION_FAILED = -11,
  ERROR_FRAME_NOT_WRITTEN = -12,
};

/*!
 * The actuator ids. These ids are only processed by ARDUINO_CENTER_ACTUATORS.
 *
 * The actuator ids are used to control the behavior of the arduino. For example send frames
 * from host with ID_ARD_ACT_STEER_SERVO or ID_ARD_ACT_SPEED_CONTR to control the steering and
 * throttle of the model car. Currently only the arduino with the id ARDUINO_CENTER_ACTUATORS
 * processes more than a ID_ARD_ACT_REQUEST frame. If a ID_ARD_ACT_REQUEST is sent to an arduino,
 * the arduino answer this request with an tInfoData.
 */
enum ACTUATOR_ID /*: uint8_t*/
{
  ID_ARD_ACT_NO_NAME = 0,
  ID_ARD_ACT_WATCHDOG = 1,
  ID_ARD_ACT_EMERGENCY_STOP = 2,
  ID_ARD_ACT_REQUEST = 3,

  ID_ARD_ACT_STEER_SERVO = 4,
  ID_ARD_ACT_SPEED_CONTR = 5,

  ID_ARD_ACT_LIGHT = 6,

  ID_ARD_DISABLE_USS = 7,
  ID_ARD_ENABLE_USS = 8,
};

/*!
 * The Watchdog data. Processed by the ARDUINO_CENTER_ACTUATORS.
 *
 * The arduino with the id ARDUINO_CENTER_ACTUATORS processes this data. It should be received
 * every half a second. Otherwise the Servos stop working.
 */
typedef struct WatchdogData
{
  /*! The triggered state 1 = true,  0 = false */
  uint8_t ui8IsTriggerd;
} tWatchdogData;

/*!
 * The emergeny data. Processed by the ARDUINO_CENTER_ACTUATORS.
 *
 * The arduino with the id ARDUINO_CENTER_ACTUATORS processes this data. If this data is
 * received. The arduino shuts down the servos.
 */
typedef struct EmergencyStopData
{
  /*! The triggered state 1 = true,  0 = false */
  uint8_t ui8IsTriggerd;
} tEmergencyStopData;

/*!
 * The servo data. Processed by the ARDUINO_CENTER_ACTUATORS.
 *
 * The arduino with the id ARDUINO_CENTER_ACTUATORS processes this data. If this data is
 * received. The arduino sets the servo position (used by steering and throttle) accordingly.
 * The range is 0...180 with 90 as zero position.
 */
typedef struct ServoData
{
  /*! The angle. Actually a servo number range is [0, 180]*/
  uint8_t ui8Angle;
} tServoData;

/*!
 * The light data. Processed by the ARDUINO_CENTER_ACTUATORS.
 *
 * The arduino with the id ARDUINO_CENTER_ACTUATORS processes this data. This data is filled
 * with a member of the LIGHT_MASK enum. To set more lights than one the enums are combined with
 * each other.
 */
typedef struct LightData
{
  /*! see enum LIGHT_MASK for IDs */
  uint8_t ui8LightMask;
} tLightData;

/*!
 * The light mask enum. Processed by the ARDUINO_CENTER_ACTUATORS.
 *
 * The arduino with the id ARDUINO_CENTER_ACTUATORS processes this data. The members of that
 * enum are used to set the ui8LightMask in tLightData.
 */
enum LIGHT_MASK /*: uint8_t*/
{
  ID_ARD_ACT_LIGHT_MASK_HEAD = 0x01,
  //const uint8_t ID_ARD_ACT_LIGHT_MASK_BACK = 0x02; //same Pin as ID_ARD_ACT_LIGHT_MASK_HEAD
  ID_ARD_ACT_LIGHT_MASK_BRAKE = 0x04,
  ID_ARD_ACT_LIGHT_MASK_TURNLEFT = 0x08,
  ID_ARD_ACT_LIGHT_MASK_TURNRIGHT = 0x10,
  ID_ARD_ACT_LIGHT_MASK_HAZARD = ID_ARD_ACT_LIGHT_MASK_TURNLEFT
      | ID_ARD_ACT_LIGHT_MASK_TURNRIGHT,
  ID_ARD_ACT_LIGHT_MASK_REVERSE = 0x20,
};

typedef bool tUpdatebit;


typedef struct frontpacket{
  tUsData us_front_right;
  tUpdatebit us_front_right_update;
  tUsData us_front_center_right;
  tUpdatebit us_front_center_right_update;
  tUsData us_front_center;
  tUpdatebit us_front_center_update;
  tUsData us_front_center_left;
  tUpdatebit us_front_center_left_update;
  tUsData us_front_left;
  tUpdatebit us_front_left_update;
  int error;
}tFrontPacket;

typedef struct centmeaspacket{
  tVoltageData volt_actuator;
  tVoltageData volt_actuator_cell1;
  tVoltageData volt_actuator_cell2;
  tVoltageData volt_sensors;
  tVoltageData volt_sensors_cell1;
  tVoltageData volt_sensors_cell2;
  tVoltageData volt_sensors_cell3;
  tVoltageData volt_sensors_cell4;
  tVoltageData volt_sensors_cell5;
  tVoltageData volt_sensors_cell6;
  tUpdatebit volt_actuator_update;
  tUpdatebit volt_actuator_cell1_update;
  tUpdatebit volt_actuator_cell2_update;
  tUpdatebit volt_sensors_update;
  tUpdatebit volt_sensors_cell1_update;
  tUpdatebit volt_sensors_cell2_update;
  tUpdatebit volt_sensors_cell3_update;
  tUpdatebit volt_sensors_cell4_update;
  tUpdatebit volt_sensors_cell5_update;
  tUpdatebit volt_sensors_cell6_update;
  int error;
}tCentMeasPacket;

typedef struct rearimupacket{
  tSensWheelData wheel_right;
  tSensWheelData wheel_left;
  tImuData imu;
  int error;
}tRearImuPacket;

typedef struct remotepacket
{
  tRemoteData remote;
  int error;
} tRemotePacket;

extern int open_ttyACM(const char *filename);
extern int map_arduino_id(uint8_t id);
extern uint8_t get_id(char buf[], int length);
extern void hexdump(char buf[], int length);
extern int read_packet(int fd, char *buf);
extern int get_Frame(char buf[], int fd);
extern uint16_t
fletcher16 (uint8_t const *data, uint8_t bytes);
extern int
send_actor_frame (int fd, uint8_t ui8ID, uint8_t ui8DataLength,
		  uint32_t ui32Timestamp, uint8_t data);
extern int
get_sensor_arduino (int fd, uint8_t address);
extern int
get_actor_arduino (int fd, uint8_t address);
extern int scan_ttyACM(uint8_t address);

extern tUsData parse_us(char buf[], int length);
extern tFrontPacket read_front_frame(int fd,    tFrontPacket packet);

extern tVoltageData parse_volt(char buf[], int length);
extern tCentMeasPacket read_center_frame(int fd,    tCentMeasPacket packet);

extern tSensWheelData parse_wheel(char buf[], int length);
extern tImuData parse_imu(char buf[], int length);
extern tRearImuPacket read_rear_imu_frame(int fd, tRearImuPacket packet);
extern tRemoteData
parse_remote (char buf[], int length);
extern tRemotePacket
read_remote_frame (int fd, tRemotePacket packet);

#endif

#endif /* SRC_OTHERFILES_ARDUINO_UTILITIES_H_ */
