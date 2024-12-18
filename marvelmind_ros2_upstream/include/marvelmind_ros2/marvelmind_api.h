#ifndef __MARVELMIND_API_H_
#define __MARVELMIND_API_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MM_API_SUCCESS 0
#define MM_API_ERROR_COMMUNICATION 1
#define MM_API_ERROR_PORT_NOT_OPENED 2
#define MM_API_ERROR_NO_LICENSE 3
#define MM_API_ERROR_NO_DATA 4
#define MM_API_ERROR_REQUEST_TOO_LARGE 256


#define MM_USB_DEVICE_ADDRESS 255

#define MM_MAX_DEVICES_COUNT 255

bool mmAPIVersion(uint32_t *version);

bool mmGetLastError(uint32_t *error);

bool mmOpenPort();
bool mmOpenPortByName(char *portName);
bool mmOpenPortUDP(char *IP_address, uint16_t port, uint16_t timeout_ms);
void mmClosePort();

typedef struct {
     uint8_t fwVerMajor;
     uint8_t fwVerMinor;
     uint8_t fwVerMinor2;
     uint8_t fwVerDeviceType;

     uint8_t fwOptions;

     uint32_t cpuId;
} MarvelmindDeviceVersion;
bool mmGetVersionAndId(uint8_t address, MarvelmindDeviceVersion *mmDevVersion);

typedef struct {
     uint8_t address;
     bool isDuplicatedAddress;
     bool isSleeping;

     uint8_t fwVerMajor;
     uint8_t fwVerMinor;
     uint8_t fwVerMinor2;
     uint8_t fwVerDeviceType;

     uint8_t fwOptions;

     uint8_t flags;
} MarvelmindDeviceInfo;
typedef struct {
    uint8_t numDevices;
    MarvelmindDeviceInfo devices[MM_MAX_DEVICES_COUNT];
} MarvelmindDevicesList;
bool mmGetDevicesList(MarvelmindDevicesList *mmDevices);

bool mmWakeDevice(uint8_t address);
bool mmSendToSleepDevice(uint8_t address);

typedef struct {
      uint32_t worktimeSec;
      int8_t rssi;
      int8_t temperature;
      uint16_t voltageMv;
      uint8_t reserved[16];
} MarvelmindBeaconTelemetry;
bool mmGetBeaconTelemetry(uint8_t address, MarvelmindBeaconTelemetry *bTele);

#define MM_LOCATIONS_VERSION 2

#define MM_LOCATIONS_PACK_SIZE 6
#define MM_USER_PAYLOAD_BUF_SIZE 128
typedef struct {
      uint8_t address;
      uint8_t headIndex;
      int32_t x_mm;
      int32_t y_mm;
      int32_t z_mm;
      uint8_t statusFlags;
      uint8_t quality;
      uint8_t reserved[2];
} MarvelmindDeviceLocation;
typedef struct {
      MarvelmindDeviceLocation pos[MM_LOCATIONS_PACK_SIZE];

      bool lastDistUpdated;
      uint8_t reserved[5];

      uint8_t userPayloadSize;
      uint8_t userPayloadBuf[MM_USER_PAYLOAD_BUF_SIZE];
} MarvelmindLocationsPack;
bool mmGetLastLocations(MarvelmindLocationsPack *posPack);

typedef struct {
      uint8_t address;
      uint8_t headIndex;
      int32_t x_mm;
      int32_t y_mm;
      int32_t z_mm;
      uint8_t statusFlags;
      uint8_t quality;
      uint8_t reserved[2];
      uint16_t angle;
      bool angleReady;
} MarvelmindDeviceLocation2;
typedef struct {
      MarvelmindDeviceLocation2 pos[MM_LOCATIONS_PACK_SIZE];

      bool lastDistUpdated;
      uint8_t reserved[5];

      uint8_t userPayloadSize;
      uint8_t userPayloadBuf[MM_USER_PAYLOAD_BUF_SIZE];
} MarvelmindLocationsPack2;
bool mmGetLastLocations2(MarvelmindLocationsPack2 *posPack);
bool mmGetLastLocations2a(MarvelmindLocationsPack2 *posPack, uint8_t options);

#define MM_DISTANCES_PACK_MAX_SIZE 16
typedef struct {
      uint8_t addressRx;
      uint8_t headRx;
      uint8_t addressTx;
      uint8_t headTx;
      uint32_t distance_mm;
      uint8_t reserved;
} MarvelmindDistance;
typedef struct {
      uint8_t numDistances;
      MarvelmindDistance distance[MM_DISTANCES_PACK_MAX_SIZE];
} MarvelmindDistances;
bool mmGetLastDistances(MarvelmindDistances *distPack);

bool mmGetUpdateRateSetting(float *updRateHz);
bool mmSetUpdateRateSetting(float *updRateHz);

bool mmAddSubmap(uint8_t submapId);
bool mmDeleteSubmap(uint8_t submapId);
bool mmFreezeSubmap(uint8_t submapId);
bool mmUnfreezeSubmap(uint8_t submapId);

#define MM_SUBMAP_BEACONS_MAX_NUM 4
#define MM_NEARBY_SUBMAPS_MAX_NUM 8
#define MM_SUBMAP_SERVICE_ZONE_MAX_POINTS 8
typedef struct {
     int16_t x;
     int16_t y;
} ServiceZonePoint;
typedef struct {
     uint8_t startingBeacon;// Starting beacon trilateration
     uint8_t startingSet_1;// Starting set of beacons
     uint8_t startingSet_2;
     uint8_t startingSet_3;
     uint8_t startingSet_4;

     bool enabled3d;// 3D navigation
     bool onlyForZ;// Only for Z coordinate

     bool limitationDistanceIsManual;// Limitation distances
     uint8_t maximumDistanceManual_m;// Maximum distance, m

     int16_t submapShiftX_cm;// Submap X shift, cm
     int16_t submapShiftY_cm;// Submap Y shift, cm
     int16_t submapShiftZ_cm;// Submap Z shift, cm
     uint16_t submapRotation_cdeg;// Submap rotation, centidegrees

     int16_t planeQw;// Plane rotation quaternion (QW,QX,QY,QZ) normalized to 10000
     int16_t planeQx;
     int16_t planeQy;
     int16_t planeQz;

     int16_t serviceZoneThickness_cm;// Service zone thickness, cm

     int16_t hedgesHeightFor2D_cm;// Hedges height in 2D mode

     bool frozen;// true - submap is frozen
     bool locked;// true - submap is locked

     bool beaconsHigher;// true - stationary beacons are higher than mobile
     bool mirrored;// true - submap is mirrored

     uint8_t beacons[MM_SUBMAP_BEACONS_MAX_NUM];// list of beacons in submap (0 - none)
     uint8_t nearbySubmaps[MM_NEARBY_SUBMAPS_MAX_NUM];// list of nearby submaps ID's (255 - none)

     uint8_t serviceZonePointsNum;// Number of service zone polygon points
     ServiceZonePoint serviceZonePolygon[MM_SUBMAP_SERVICE_ZONE_MAX_POINTS];
} MarvelmindSubmapSettings;
bool mmGetSubmapSettings(uint8_t submapId, MarvelmindSubmapSettings *submapSettings);
bool mmSetSubmapSettings(uint8_t submapId, MarvelmindSubmapSettings *submapSettings);

#define MM_SENSOR_RX1 0
#define MM_SENSOR_RX2 1
#define MM_SENSOR_RX3 2
#define MM_SENSOR_RX4 3
#define MM_SENSOR_RX5 4
#define MM_US_SENSORS_NUM 5

#define MM_US_FILTER_19KHZ 0
#define MM_US_FILTER_25KHZ 1
#define MM_US_FILTER_31KHZ 2
#define MM_US_FILTER_37KHZ 3
#define MM_US_FILTER_45KHZ 4

typedef struct {
     uint16_t txFrequency_hz;
     uint8_t txPeriodsNumber;

     bool rxAmplifierAGC;
     uint16_t rxAmplificationManual;

     bool sensorsNormal[MM_US_SENSORS_NUM];
     bool sensorsFrozen[MM_US_SENSORS_NUM];

     uint8_t rxDSPFilterIndex;
} MarvelmindUltrasoundSettings;
bool mmGetUltrasoundSettings(uint8_t address, MarvelmindUltrasoundSettings *usSettings);
bool mmSetUltrasoundSettings(uint8_t address, MarvelmindUltrasoundSettings *usSettings);

bool mmEraseMap();
bool mmSetDefaultSettings(uint8_t address);

bool mmFreezeMap();
bool mmUnfreezeMap();

bool mmBeaconsToAxes(uint8_t address_0, uint8_t address_x, uint8_t address_y);

bool mmReadFlashDump(uint32_t offset, uint32_t size, void *pdata);
bool mmWriteFlashDump(uint32_t offset, uint32_t size, void *pdata);

bool mmResetDevice(uint8_t address);

bool mmGetAirTemperature(int8_t *ptemperature);
bool mmSetAirTemperature(int8_t temperature);

bool mmSetBeaconLocation(uint8_t address, int32_t x_mm, int32_t y_mm, int32_t z_mm);
bool mmSetBeaconsDistance(uint8_t address_1, uint8_t address_2, int32_t distance_mm);

bool mmGetHedgeHeight(uint8_t address, int32_t *pheight_mm);
bool mmSetHedgeHeight(uint8_t address, int32_t height_mm);

bool mmGetBeaconHeight(uint8_t address, uint8_t submapId, int32_t *pheight_mm);
bool mmSetBeaconHeight(uint8_t address, uint8_t submapId, int32_t height_mm);

typedef struct {
     bool rtpEnabled;
     uint8_t rtpForward;
     uint8_t rtpBackward;
     uint8_t reserved0;
     uint8_t reserved1;
} MarvelmindRealtimePlayerSettings;
bool mmGetRealtimePlayerSettings(uint8_t address, MarvelmindRealtimePlayerSettings *rtp);
bool mmSetRealtimePlayerSettings(uint8_t address, MarvelmindRealtimePlayerSettings *rtp);

typedef struct {
     int32_t latitude_x100ndeg;
     int32_t longitude_x100ndeg;
} MarvelmindGeoreferencingSettings;
bool mmGetGeoreferencingSettings(MarvelmindGeoreferencingSettings *gr);
bool mmSetGeoreferencingSettings(MarvelmindGeoreferencingSettings *gr);

typedef struct {
     uint8_t mode;
     uint8_t reserved[7];
} MarvelmindUpdatePositionsMode;
bool mmGetUpdatePositionsMode(MarvelmindUpdatePositionsMode *ups);
bool mmSetUpdatePositionsMode(MarvelmindUpdatePositionsMode *ups);

typedef struct {
     uint8_t reserved[8];
} MarvelmindUpdatePositionsCommand;
bool mmSendUpdatePositionsCommand(MarvelmindUpdatePositionsCommand *upc);

typedef struct {
     uint8_t alarmMode;
     uint8_t zoneIndex;
     uint8_t reserved[6];
} MarvelmindAlarmState;
bool mmSendAlarmState(uint8_t address, MarvelmindAlarmState *mas);

typedef struct {
    uint8_t dsize;
    uint8_t data[256];
} MarvelmindSendUserPayload;
bool mmSendUserPayload(uint8_t address, MarvelmindSendUserPayload *mup);

typedef struct {
    uint8_t address;
    int64_t timestamp;
    uint8_t dsize;
    uint8_t data[256];
} MarvelmindGetUserPayload;
bool mmGetUserPayload(MarvelmindGetUserPayload *mup);

typedef struct {
     uint8_t mode;
     uint8_t moveType;
     uint8_t levelPercents;
} MarvelmindMotorsSettings;
bool mmSetMotorsControl(uint8_t address, MarvelmindMotorsSettings *motorsCtrl);


typedef struct {
     uint8_t itemIndex;
     uint8_t totalItemsNum;

     uint8_t opCode;
     int16_t param1;
     int16_t param2;
     int16_t param3;
} MarvelmindRobotProgramItem;
bool mmSetRobotProgramItem(uint8_t address, MarvelmindRobotProgramItem *progItem);

typedef struct {
     uint8_t commandId;

     int16_t param1;
     int16_t param2;
     int16_t param3;
} MarvelmindRobotCommand;
bool mmSetRobotCommand(uint8_t address, MarvelmindRobotCommand *robotCommand);

typedef struct {
     int32_t x_mm;
     int32_t y_mm;
     int32_t z_mm;

     uint16_t angle;

     uint8_t reserved[18];
} MarvelmindRobotPosition;
bool mmSetRobotPosition(uint8_t address, MarvelmindRobotPosition *robotPosition);

typedef struct {
     uint8_t data[64];
} MarvelmindRobotTelemetry;
bool mmGetRobotTelemetry(uint8_t address, MarvelmindRobotTelemetry *telemetry);

typedef struct {
     uint8_t page;
     uint8_t data[32];
} MarvelmindRobotSettings;
bool mmGetRobotSettings(uint8_t address, MarvelmindRobotSettings *robotSettings);
bool mmSetRobotSettings(uint8_t address, MarvelmindRobotSettings *robotSettings);

typedef struct {
  uint16_t batteryVoltage_x10mv;
  uint16_t totalCurrent_x10ma;
  uint16_t motorsCurrent_x10ma;
  uint8_t batteryCapacity_per;
  uint32_t timestamp_ms;
  uint8_t flags;

  uint8_t reserved[20];
} MarvelmindRobotV100Power;
bool mmRobotV100GetPower(MarvelmindRobotV100Power *power);

typedef struct {
  int32_t leftEncoderPath_cm;
  int32_t rightEncoderPath_cm;
  uint32_t timestamp_ms;

  uint8_t reserved[20];
} MarvelmindRobotV100Encoders;
bool mmRobotV100GetEncoders(MarvelmindRobotV100Encoders *encoders);

#define RV100_LIDARS_NUM 12
typedef struct {
    uint16_t range_mm;
    uint8_t status;
} RobotV100LidarState;
typedef struct {
  RobotV100LidarState lidars[RV100_LIDARS_NUM];
  uint32_t timestamp_ms;

  uint8_t reserved[32];
} MarvelmindRobotV100Lidars;
bool mmRobotV100GetLidars(MarvelmindRobotV100Lidars *lidars);

typedef struct {
  float x_m;
  float y_m;
  float z_m;
  float yaw_angle_deg;
  uint8_t flags;
  uint32_t timestamp_ms;

  uint8_t reserved[13];
} MarvelmindRobotV100Location;
bool mmRobotV100GetLocation(MarvelmindRobotV100Location *pos);

typedef struct {
  int16_t ax_mg;
  int16_t ay_mg;
  int16_t az_mg;
  int16_t gx;
  int16_t gy;
  int16_t gz;

  uint32_t timestamp_ms;

  uint8_t reserved[16];
} MarvelmindRobotV100RawIMU;
bool mmRobotV100GetRawIMU(MarvelmindRobotV100RawIMU *rawIMU);

typedef struct {
  uint8_t motorsMode;
  uint8_t leftMotorSpeed;
  uint8_t rightMotorSpeed;
  uint8_t leftMotorFlags;
  uint8_t rightMotorFlags;

  uint8_t reserved[11];
} MarvelmindRobotV100Motors;
bool mmRobotV100SetMotors(MarvelmindRobotV100Motors *motors);

bool mmDeviceIsModem(uint8_t deviceType);
bool mmDeviceIsBeacon(uint8_t deviceType);
bool mmDeviceIsHedgehog(uint8_t deviceType);
bool mmDeviceIsRobot(uint8_t deviceType);

#define MM_API_ID_API_VERSION 1
#define MM_API_ID_GET_LAST_ERROR 2
#define MM_API_ID_OPEN_PORT 3
#define MM_API_ID_OPEN_PORT_BY_NAME 4
#define MM_API_ID_OPEN_PORT_UDP 5
#define MM_API_ID_CLOSE_PORT 6
#define MM_API_ID_GET_VERSION_AND_ID 7
#define MM_API_ID_GET_DEVICES_LIST 8
#define MM_API_ID_WAKE_DEVICE 9
#define MM_API_ID_SLEEP_DEVICE 10
#define MM_API_ID_GET_BEACON_TELEMETRY 11
#define MM_API_ID_GET_LAST_LOCATIONS2 13
#define MM_API_ID_GET_LAST_DISTANCES 14
#define MM_API_ID_GET_UPDATE_RATE 15
#define MM_API_ID_SET_UPDATE_RATE 16
#define MM_API_ID_ADD_SUBMAP 17
#define MM_API_ID_DELETE_SUBMAP 18
#define MM_API_ID_FREEZE_SUBMAP 19
#define MM_API_ID_UNFREEZE_SUBMAP 20
#define MM_API_ID_ERASE_MAP 30
#define MM_API_ID_SET_DEFAULT 31
#define MM_API_ID_FREEZE_MAP 32
#define MM_API_ID_UNFREEZE_MAP 33
#define MM_API_ID_BEACONS_TO_AXES 34
#define MM_API_ID_READ_FLASH_DUMP 35
#define MM_API_ID_WRITE_FLASH_DUMP 36
#define MM_API_ID_RESET_DEVICE 37
#define MM_API_ID_GET_AIR_TEMPERATURE 38
#define MM_API_ID_SET_AIR_TEMPERATURE 39
#define MM_API_ID_SET_BEACON_LOCATION 40
#define MM_API_ID_SET_BEACONS_DISTANCE 41
#define MM_API_ID_GET_HEDGE_HEIGHT 42
#define MM_API_ID_SET_HEDGE_HEIGHT 43
#define MM_API_ID_GET_BEACON_HEIGHT 44
#define MM_API_ID_SET_BEACON_HEIGHT 45
#define MM_API_ID_GET_RTP_SETTINGS 46
#define MM_API_ID_SET_RTP_SETTINGS 47
#define MM_API_ID_GET_GEOREFERENCING 48
#define MM_API_ID_SET_GEOREFERENCING 49
#define MM_API_ID_GET_UPDATE_POSITIONS_MODE 50
#define MM_API_ID_SET_UPDATE_POSITIONS_MODE 51
#define MM_API_ID_SEND_UPDATE_POSITIONS_COMMAND 52
#define MM_API_ID_SET_ALARM_STATE 53
#define MM_API_ID_GET_USER_PAYLOAD 54
#define MM_API_ID_SET_USER_PAYLOAD 55
#define MM_API_ID_DEVICE_IS_MODEM 56
#define MM_API_ID_DEVICE_IS_BEACON 57
#define MM_API_ID_DEVICE_IS_HEDGEHOG 58
#define MM_API_ID_DEVICE_IS_ROBOT 59


bool marvelmindAPICall(int64_t command_id, uint8_t* request, uint32_t request_size, uint8_t* response, uint32_t* response_size, int32_t* error_code);

void marvelmindAPILoad(char *dll_filepath);
void marvelmindAPIFree();

#ifdef __cplusplus
}
#endif

#endif // __MARVELMIND_API_H_
