#include "marvelmind_ros2/marvelmind_api.h"
#ifdef WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif
#include <string.h>

#ifdef WIN32
HINSTANCE mmLibrary;
#define MM_STDCALL1 
#define MM_STDCALL2 __stdcall
#else
void* mmLibrary;
#if defined(__x86_64__)
#define MM_STDCALL1
#else
#define MM_STDCALL1 __attribute__((stdcall))
#endif
#define MM_STDCALL2
#endif

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_api_version)(void *pdata);
static pt_mm_api_version pmm_api_version= NULL;
bool mmAPIVersion(uint32_t *version) {
  if (pmm_api_version == NULL)
    return false;

  uint8_t buf[8];
  bool res= (*pmm_api_version)(&buf[0]);

  *version= *((uint32_t *) &buf[0]);

  return res;
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_last_error)(void *pdata);
static pt_mm_get_last_error pmm_get_last_error= NULL;
bool mmGetLastError(uint32_t *error) {
  if (pmm_get_last_error == NULL)
    return false;

  uint8_t buf[8];
  bool res= (*pmm_get_last_error)(&buf[0]);

  *error= *((uint32_t *) &buf[0]);

  return res;
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_open_port)(void);
static pt_mm_open_port pmm_open_port= NULL;
bool mmOpenPort() {
  if (pmm_open_port == NULL)
    return false;

  return (*pmm_open_port)();
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_open_port_by_name)(void *pdata);
static pt_mm_open_port_by_name pmm_open_port_by_name= NULL;
bool mmOpenPortByName(char *portName) {
  if (pmm_open_port_by_name == NULL)
    return false;

  uint8_t buf[255];
  uint8_t i;
  for(i=0;i<255;i++) {
    buf[i]= portName[i];
    if (buf[i] == 0)
        break;
  }

  return (*pmm_open_port_by_name)(&buf[0]);
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_open_port_udp)(void *pdata);
static pt_mm_open_port_udp pmm_open_port_udp= NULL;
bool mmOpenPortUDP(char *IP_address, uint16_t port, uint16_t timeout_ms) {
  if (pmm_open_port_udp == NULL)
    return false;

  uint8_t buf[300];
  uint16_t ofs;
  uint8_t i;

  ofs= 0;

  buf[ofs++]= port&0xff;
  buf[ofs++]= (port>>8)&0xff;

  buf[ofs++]= timeout_ms&0xff;
  buf[ofs++]= (timeout_ms>>8)&0xff;

  buf[ofs++]= 0;
  buf[ofs++]= 0;// reserved

  for(i=0;i<255;i++) {
    buf[ofs+i]= IP_address[i];
    if (buf[ofs+i] == 0)
        break;
  }

  return (*pmm_open_port_udp)(&buf[0]);
}


//////

typedef void MM_STDCALL1 (MM_STDCALL2 *pt_mm_close_port)(void);
static pt_mm_close_port pmm_close_port= NULL;
void mmClosePort() {
  if (pmm_close_port == NULL)
    return;

  (*pmm_close_port)();
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_version_and_id)(uint8_t address, void *pdata);
static pt_mm_get_version_and_id pmm_get_version_and_id= NULL;
bool mmGetVersionAndId(uint8_t address, MarvelmindDeviceVersion *mmDevVersion) {
  if (pmm_get_version_and_id == NULL)
    return false;

  uint8_t buf[128];
  bool res= (*pmm_get_version_and_id)(address, &buf[0]);

  mmDevVersion->fwVerMajor= buf[0];
  mmDevVersion->fwVerMinor= buf[1];
  mmDevVersion->fwVerMinor2= buf[2];
  mmDevVersion->fwVerDeviceType= buf[3];
  mmDevVersion->fwOptions= buf[4];

  mmDevVersion->cpuId= *((uint32_t *) &buf[5]);

  return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_devices_list)(void *pdata);
static pt_mm_get_devices_list pmm_get_devices_list= NULL;
bool mmGetDevicesList(MarvelmindDevicesList *mmDevices) {
  if (pmm_get_devices_list == NULL)
    return false;

  uint8_t buf[(MM_MAX_DEVICES_COUNT+1)*10];

  bool res= (*pmm_get_devices_list)(&buf[0]);

  if (res) {
    uint8_t i;
    MarvelmindDeviceInfo *devPtr;

    mmDevices->numDevices= buf[0];

    uint32_t ofs= 1;
    for(i=0;i<mmDevices->numDevices;i++) {
        devPtr= &mmDevices->devices[i];

        devPtr->address= buf[ofs+0];
        devPtr->isDuplicatedAddress= (bool) buf[ofs+1];
        devPtr->isSleeping= (bool) buf[ofs+2];

        devPtr->fwVerMajor= buf[ofs+3];
        devPtr->fwVerMinor= buf[ofs+4];
        devPtr->fwVerMinor2= buf[ofs+5];
        devPtr->fwVerDeviceType= buf[ofs+6];

        devPtr->fwOptions= buf[ofs+7];

        devPtr->flags= buf[ofs+8];

        ofs+= 9;
    }
  }

  return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_wake_device)(uint8_t address);
static pt_mm_wake_device pmm_wake_device= NULL;
bool mmWakeDevice(uint8_t address) {
    if (pmm_wake_device == NULL)
        return false;

    bool res= (*pmm_wake_device)(address);

    return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_sleep_device)(uint8_t address);
static pt_mm_sleep_device pmm_sleep_device= NULL;
bool mmSendToSleepDevice(uint8_t address) {
    if (pmm_sleep_device == NULL)
        return false;

    bool res= (*pmm_sleep_device)(address);

    return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_beacon_tele) (uint8_t address, void *pdata);
static pt_mm_get_beacon_tele pmm_get_beacon_tele= NULL;
bool mmGetBeaconTelemetry(uint8_t address, MarvelmindBeaconTelemetry *bTele) {
    if (pmm_get_beacon_tele == NULL)
        return false;

    uint8_t buf[128];
    uint8_t i;
    bool res= (*pmm_get_beacon_tele)(address, (void *) &buf[0]);

    bTele->worktimeSec= *((uint32_t *) &buf[0]);
    bTele->rssi= *((int8_t *) &buf[4]);
    bTele->temperature= *((int8_t *) &buf[5]);
    bTele->voltageMv= *((uint16_t *) &buf[6]);

    for(i=0;i<16;i++) {
        bTele->reserved[i]= buf[8+i];
    }

    return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_last_locations) (void *pdata);
static pt_mm_get_last_locations pmm_get_last_locations= NULL;
bool mmGetLastLocations(MarvelmindLocationsPack *posPack) {
    if (pmm_get_last_locations == NULL)
        return false;

    uint8_t buf[512];
    uint8_t i;
    bool res= (*pmm_get_last_locations)((void *) &buf[0]);

    uint16_t ofs= 0;
    MarvelmindDeviceLocation *ppos;
    for(i=0;i<MM_LOCATIONS_PACK_SIZE;i++) {
        ppos= &posPack->pos[i];

        ppos->address= buf[ofs+0];
        ppos->headIndex= buf[ofs+1];

        ppos->x_mm= *((int32_t *) &buf[ofs+2]);
        ppos->y_mm= *((int32_t *) &buf[ofs+6]);
        ppos->z_mm= *((int32_t *) &buf[ofs+10]);

        ppos->statusFlags= buf[ofs+14];
        ppos->quality= buf[ofs+15];

        ppos->reserved[0]= buf[ofs+16];
        ppos->reserved[1]= buf[ofs+17];

        ofs+= 18;
    }

    posPack->lastDistUpdated= buf[ofs++];
    for(i=0;i<5;i++) {
        posPack->reserved[i]= buf[ofs++];
    }

    posPack->userPayloadSize= buf[ofs++];
    uint8_t n= posPack->userPayloadSize;
    if (n>MM_USER_PAYLOAD_BUF_SIZE) {
        n= MM_USER_PAYLOAD_BUF_SIZE;
    }
    for(i=0;i<n;i++) {
        posPack->userPayloadBuf[i]= buf[ofs++];
    }

    return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_last_locations2) (void *pdata);
static pt_mm_get_last_locations2 pmm_get_last_locations2= NULL;
bool mmGetLastLocations2(MarvelmindLocationsPack2 *posPack) {
    if (pmm_get_last_locations2 == NULL)
        return false;

    uint8_t buf[512];
    uint8_t i;
    bool res= (*pmm_get_last_locations2)((void *) &buf[0]);

    uint16_t ofs= 0;
    MarvelmindDeviceLocation2 *ppos;
    for(i=0;i<MM_LOCATIONS_PACK_SIZE;i++) {
        ppos= &posPack->pos[i];

        ppos->address= buf[ofs+0];
        ppos->headIndex= buf[ofs+1];

        ppos->x_mm= *((int32_t *) &buf[ofs+2]);
        ppos->y_mm= *((int32_t *) &buf[ofs+6]);
        ppos->z_mm= *((int32_t *) &buf[ofs+10]);

        ppos->statusFlags= buf[ofs+14];
        ppos->quality= buf[ofs+15];

        ppos->reserved[0]= buf[ofs+16];
        ppos->reserved[1]= buf[ofs+17];

        ppos->angle= *((uint16_t *) &buf[ofs+18]);
        ppos->angleReady= ((ppos->angle&0x1000) == 0);

        ofs+= 20;
    }

    posPack->lastDistUpdated= buf[ofs++];
    for(i=0;i<5;i++) {
        posPack->reserved[i]= buf[ofs++];
    }

    posPack->userPayloadSize= buf[ofs++];
    uint8_t n= posPack->userPayloadSize;
    if (n>MM_USER_PAYLOAD_BUF_SIZE) {
        n= MM_USER_PAYLOAD_BUF_SIZE;
    }
    for(i=0;i<n;i++) {
        posPack->userPayloadBuf[i]= buf[ofs++];
    }

    return res;
}

bool mmGetLastLocations2a(MarvelmindLocationsPack2 *posPack, uint8_t options) {
    if (pmm_get_last_locations2 == NULL)
        return false;

    uint8_t buf[512];
    uint8_t i;

    buf[0]= options;
    bool res= (*pmm_get_last_locations2)((void *) &buf[0]);

    uint16_t ofs= 0;
    MarvelmindDeviceLocation2 *ppos;
    for(i=0;i<MM_LOCATIONS_PACK_SIZE;i++) {
        ppos= &posPack->pos[i];

        ppos->address= buf[ofs+0];
        ppos->headIndex= buf[ofs+1];

        ppos->x_mm= *((int32_t *) &buf[ofs+2]);
        ppos->y_mm= *((int32_t *) &buf[ofs+6]);
        ppos->z_mm= *((int32_t *) &buf[ofs+10]);

        ppos->statusFlags= buf[ofs+14];
        ppos->quality= buf[ofs+15];

        ppos->reserved[0]= buf[ofs+16];
        ppos->reserved[1]= buf[ofs+17];

        ppos->angle= *((uint16_t *) &buf[ofs+18]);
        ppos->angleReady= ((ppos->angle&0x1000) == 0);

        ofs+= 20;
    }

    posPack->lastDistUpdated= buf[ofs++];
    for(i=0;i<5;i++) {
        posPack->reserved[i]= buf[ofs++];
    }

    posPack->userPayloadSize= buf[ofs++];
    uint8_t n= posPack->userPayloadSize;
    if (n>MM_USER_PAYLOAD_BUF_SIZE) {
        n= MM_USER_PAYLOAD_BUF_SIZE;
    }
    for(i=0;i<n;i++) {
        posPack->userPayloadBuf[i]= buf[ofs++];
    }

    return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_last_distances) (void *pdata);
static pt_mm_get_last_distances pmm_get_last_distances= NULL;
bool mmGetLastDistances(MarvelmindDistances *distPack) {
    if (pmm_get_last_distances == NULL)
        return false;

    uint8_t buf[512];
    uint8_t i;
    bool res= (*pmm_get_last_distances)((void *) &buf[0]);

    distPack->numDistances= buf[0];
    if (distPack->numDistances > MM_DISTANCES_PACK_MAX_SIZE) {
        distPack->numDistances= MM_DISTANCES_PACK_MAX_SIZE;
    }

    uint16_t ofs= 1;
    MarvelmindDistance *pdist;
    for(i=0;i<distPack->numDistances;i++) {
        pdist= &distPack->distance[i];

        pdist->addressRx= buf[ofs+0];
        pdist->headRx= buf[ofs+1];
        pdist->addressTx= buf[ofs+2];
        pdist->headTx= buf[ofs+3];

        pdist->distance_mm= *((uint32_t *) &buf[ofs+4]);

        pdist->reserved= buf[ofs+8];

        ofs+= 9;
    }

    return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_update_rate_setting) (void *pdata);
static pt_mm_get_update_rate_setting pmm_get_update_rate_setting= NULL;
bool mmGetUpdateRateSetting(float *updRateHz) {
    if (pmm_get_update_rate_setting == NULL)
        return false;

    uint8_t buf[8];
    bool res= (*pmm_get_update_rate_setting)((void *) &buf[0]);

    if (res) {
       uint32_t updRate_mHz= *((uint32_t *) &buf[0]);

       *updRateHz= updRate_mHz/1000.0f;
    }

    return res;
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_update_rate_setting) (void *pdata);
static pt_mm_set_update_rate_setting pmm_set_update_rate_setting= NULL;
bool mmSetUpdateRateSetting(float *updRateHz) {
    if (pmm_set_update_rate_setting == NULL)
        return false;

    uint8_t buf[8];

    uint32_t updRate_mHz= (uint32_t) ((*updRateHz)*1000.0f);
    *((uint32_t *) &buf[0])= updRate_mHz;

    bool res= (*pmm_set_update_rate_setting)((void *) &buf[0]);

    return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_add_submap) (uint8_t submapId);
static pt_mm_add_submap pmm_add_submap= NULL;
bool mmAddSubmap(uint8_t submapId) {
    if (pmm_add_submap == NULL)
        return false;

     bool res= (*pmm_add_submap)(submapId);

     return res;
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_delete_submap) (uint8_t submapId);
static pt_mm_delete_submap pmm_delete_submap= NULL;
bool mmDeleteSubmap(uint8_t submapId) {
    if (pmm_delete_submap == NULL)
        return false;

     bool res= (*pmm_delete_submap)(submapId);

     return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_freeze_submap) (uint8_t submapId);
static pt_mm_freeze_submap pmm_freeze_submap= NULL;
bool mmFreezeSubmap(uint8_t submapId) {
    if (pmm_freeze_submap == NULL)
        return false;

    bool res= (*pmm_freeze_submap)(submapId);

    return res;
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_unfreeze_submap) (uint8_t submapId);
static pt_mm_unfreeze_submap pmm_unfreeze_submap= NULL;
bool mmUnfreezeSubmap(uint8_t submapId) {
    if (pmm_unfreeze_submap == NULL)
        return false;

    bool res= (*pmm_unfreeze_submap)(submapId);

    return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_submap_settings) (uint8_t submapId, void *pdata);
static pt_mm_get_submap_settings pmm_get_submap_settings= NULL;
bool mmGetSubmapSettings(uint8_t submapId, MarvelmindSubmapSettings *submapSettings) {
    if (pmm_get_submap_settings == NULL)
        return false;

    uint8_t buf[512];
    uint8_t i;
    bool res= (*pmm_get_submap_settings)(submapId, (void *) &buf[0]);

    submapSettings->startingBeacon= buf[0];
    submapSettings->startingSet_1= buf[1];
    submapSettings->startingSet_2= buf[2];
    submapSettings->startingSet_3= buf[3];
    submapSettings->startingSet_4= buf[4];

    submapSettings->enabled3d= (bool) buf[5];
    submapSettings->onlyForZ= (bool) buf[6];

    submapSettings->limitationDistanceIsManual= (bool) buf[7];
    submapSettings->maximumDistanceManual_m= buf[8];

    submapSettings->submapShiftX_cm= *((int16_t *) &buf[9]);
    submapSettings->submapShiftY_cm= *((int16_t *) &buf[11]);
    submapSettings->submapShiftZ_cm= *((int16_t *) &buf[13]);
    submapSettings->submapRotation_cdeg= *((uint16_t *) &buf[15]);

    submapSettings->planeQw= *((int16_t *) &buf[17]);
    submapSettings->planeQx= *((int16_t *) &buf[19]);
    submapSettings->planeQy= *((int16_t *) &buf[21]);
    submapSettings->planeQz= *((int16_t *) &buf[23]);

    submapSettings->serviceZoneThickness_cm= *((int16_t *) &buf[25]);

    submapSettings->hedgesHeightFor2D_cm= *((int16_t *) &buf[27]);

    submapSettings->frozen= (bool) buf[29];
    submapSettings->locked= (bool) buf[30];

    submapSettings->beaconsHigher= (bool) buf[31];
    submapSettings->mirrored= (bool) buf[32];

    uint8_t ofs= 33;
    for(i=0;i<MM_SUBMAP_BEACONS_MAX_NUM;i++) {
        submapSettings->beacons[i]= buf[ofs+i];
    }
    ofs+= MM_SUBMAP_BEACONS_MAX_NUM;

    for(i=0;i<MM_NEARBY_SUBMAPS_MAX_NUM;i++) {
        submapSettings->nearbySubmaps[i]= buf[ofs+i];
    }
    ofs+= MM_NEARBY_SUBMAPS_MAX_NUM;

    submapSettings->serviceZonePointsNum= buf[ofs++];
    for(i=0;i<MM_SUBMAP_SERVICE_ZONE_MAX_POINTS;i++) {
        submapSettings->serviceZonePolygon[i].x= *((int16_t *) &buf[ofs]);
        submapSettings->serviceZonePolygon[i].y= *((int16_t *) &buf[ofs+2]);

        ofs+= 4;
    }

    return res;
}


typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_submap_settings) (uint8_t submapId, void *pdata);
static pt_mm_set_submap_settings pmm_set_submap_settings= NULL;
bool mmSetSubmapSettings(uint8_t submapId, MarvelmindSubmapSettings *submapSettings) {
    if (pmm_set_submap_settings == NULL)
        return false;

    uint8_t buf[512];
    uint8_t i;

    buf[0]= submapSettings->startingBeacon;
    buf[1]= submapSettings->startingSet_1;
    buf[2]= submapSettings->startingSet_2;
    buf[3]= submapSettings->startingSet_3;
    buf[4]= submapSettings->startingSet_4;

    buf[5]= (uint8_t) submapSettings->enabled3d;
    buf[6]= (uint8_t) submapSettings->onlyForZ;

    buf[7]= (uint8_t) submapSettings->limitationDistanceIsManual;
    buf[8]= submapSettings->maximumDistanceManual_m;

    *((int16_t *) &buf[9])= submapSettings->submapShiftX_cm;
    *((int16_t *) &buf[11])= submapSettings->submapShiftY_cm;
    *((int16_t *) &buf[13])= submapSettings->submapShiftZ_cm;
    *((uint16_t *) &buf[15])= submapSettings->submapRotation_cdeg;

    *((int16_t *) &buf[17])= submapSettings->planeQw;
    *((int16_t *) &buf[19])= submapSettings->planeQx;
    *((int16_t *) &buf[21])= submapSettings->planeQy;
    *((int16_t *) &buf[23])= submapSettings->planeQz;

    *((int16_t *) &buf[25])= submapSettings->serviceZoneThickness_cm;

    *((int16_t *) &buf[27])= submapSettings->hedgesHeightFor2D_cm;

    buf[29]= (uint8_t) submapSettings->frozen;
    buf[30]= (uint8_t) submapSettings->locked;

    buf[31]= (uint8_t) submapSettings->beaconsHigher;
    buf[32]= (uint8_t) submapSettings->mirrored;

    uint8_t ofs= 33;
    for(i=0;i<MM_SUBMAP_BEACONS_MAX_NUM;i++) {
        buf[ofs+i]= submapSettings->beacons[i];
    }
    ofs+= MM_SUBMAP_BEACONS_MAX_NUM;

    for(i=0;i<MM_NEARBY_SUBMAPS_MAX_NUM;i++) {
        buf[ofs+i]= submapSettings->nearbySubmaps[i];
    }
    ofs+= MM_NEARBY_SUBMAPS_MAX_NUM;

    buf[ofs++]= submapSettings->serviceZonePointsNum;
    for(i=0;i<MM_SUBMAP_SERVICE_ZONE_MAX_POINTS;i++) {
        *((int16_t *) &buf[ofs])= submapSettings->serviceZonePolygon[i].x;
        *((int16_t *) &buf[ofs+2])= submapSettings->serviceZonePolygon[i].y;

        ofs+= 4;
    }

    bool res= (*pmm_set_submap_settings)(submapId, (void *) &buf[0]);

    return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_ultrasound_settings) (uint8_t address, void *pdata);
static pt_mm_get_ultrasound_settings pmm_get_ultrasound_settings= NULL;
bool mmGetUltrasoundSettings(uint8_t address, MarvelmindUltrasoundSettings *usSettings) {
    if (pmm_get_ultrasound_settings == NULL)
        return false;

    uint8_t buf[64];
    uint8_t i;
    bool res= (*pmm_get_ultrasound_settings)(address, (void *) &buf[0]);

    if (res) {
        usSettings->txFrequency_hz= *((uint16_t *) &buf[0]);
        usSettings->txPeriodsNumber= buf[2];

        usSettings->rxAmplifierAGC= (bool) buf[3];
        usSettings->rxAmplificationManual= *((uint16_t *) &buf[4]);

        for(i=0;i<MM_US_SENSORS_NUM;i++) {
            usSettings->sensorsNormal[i]= (bool) buf[6+i];
        }
        for(i=0;i<MM_US_SENSORS_NUM;i++) {
            usSettings->sensorsFrozen[i]= (bool) buf[11+i];
        }

        usSettings->rxDSPFilterIndex= buf[16];
    }

    return res;
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_ultrasound_settings) (uint8_t address, void *pdata);
static pt_mm_set_ultrasound_settings pmm_set_ultrasound_settings= NULL;
bool mmSetUltrasoundSettings(uint8_t address, MarvelmindUltrasoundSettings *usSettings) {
    if (pmm_set_ultrasound_settings == NULL)
        return false;

    uint8_t buf[64];
    uint8_t i;

    *((uint16_t *) &buf[0])= usSettings->txFrequency_hz;
    buf[2]= usSettings->txPeriodsNumber;

    buf[3]= (uint8_t) usSettings->rxAmplifierAGC;
    *((uint16_t *) &buf[4])= usSettings->rxAmplificationManual;

    for(i=0;i<MM_US_SENSORS_NUM;i++) {
        buf[6+i]= (uint8_t) usSettings->sensorsNormal[i];
    }
    for(i=0;i<MM_US_SENSORS_NUM;i++) {
        buf[11+i]= (uint8_t) usSettings->sensorsFrozen[i];
    }

    buf[16]= usSettings->rxDSPFilterIndex;

    bool res= (*pmm_set_ultrasound_settings)(address, (void *) &buf[0]);

    return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_erase_map) ();
static pt_mm_erase_map pmm_erase_map= NULL;
bool mmEraseMap() {
    if (pmm_erase_map == NULL)
        return false;

    return (*pmm_erase_map)();
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_default_settings) (uint8_t address);
static pt_mm_set_default_settings pmm_set_default_settings= NULL;
bool mmSetDefaultSettings(uint8_t address) {
    if (pmm_set_default_settings == NULL)
        return false;

    return (*pmm_set_default_settings)(address);
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_freeze_map) ();
static pt_mm_freeze_map pmm_freeze_map= NULL;
bool mmFreezeMap() {
    if (pmm_freeze_map == NULL)
        return false;

    return (*pmm_freeze_map)();
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_unfreeze_map) ();
static pt_mm_unfreeze_map pmm_unfreeze_map= NULL;
bool mmUnfreezeMap() {
    if (pmm_unfreeze_map == NULL)
        return false;

    return (*pmm_unfreeze_map)();
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_beacons_to_axes) (void *pdata);
static pt_mm_beacons_to_axes pmm_beacons_to_axes= NULL;
bool mmBeaconsToAxes(uint8_t address_0, uint8_t address_x, uint8_t address_y) {
    if (pmm_beacons_to_axes == NULL)
        return false;

    uint8_t buf[64];

    buf[0]= address_0;
    buf[1]= address_x;
    buf[2]= address_y;

    return (*pmm_beacons_to_axes)((void *) &buf[0]);
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_read_flash_dump) (uint32_t offset, uint32_t size, void *pdata);
static pt_read_flash_dump pmm_read_flash_dump= NULL;
bool mmReadFlashDump(uint32_t offset, uint32_t size, void *pdata) {
    if (pmm_read_flash_dump == NULL)
        return false;

    return (*pmm_read_flash_dump)(offset, size, pdata);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_write_flash_dump) (uint32_t offset, uint32_t size, void *pdata);
static pt_write_flash_dump pmm_write_flash_dump= NULL;
bool mmWriteFlashDump(uint32_t offset, uint32_t size, void *pdata) {
    if (pmm_write_flash_dump == NULL)
        return false;

    return (*pmm_write_flash_dump)(offset, size, pdata);
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_reset_device) (uint8_t address);
static pt_mm_reset_device pmm_reset_device= NULL;
bool mmResetDevice(uint8_t address) {
    if (pmm_reset_device == NULL)
        return false;

    return (*pmm_reset_device)(address);
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_air_temperature) (void *pdata);
static pt_mm_get_air_temperature pmm_get_air_temperature= NULL;
bool mmGetAirTemperature(int8_t *ptemperature) {
    if (pmm_get_air_temperature == NULL)
        return false;

    uint8_t buf[64];

    bool res= (*pmm_get_air_temperature)((void *) &buf[0]);
    if (res) {
      *ptemperature= (int8_t) buf[0];
    }

    return res;
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_air_temperature) (void *pdata);
static pt_mm_set_air_temperature pmm_set_air_temperature= NULL;
bool mmSetAirTemperature(int8_t temperature) {
    if (pmm_set_air_temperature == NULL)
        return false;

    uint8_t buf[64];
    buf[0]= temperature;

    return (*pmm_set_air_temperature)((void *) &buf[0]);
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_beacon_location) (uint8_t address, void *pdata);
static pt_mm_set_beacon_location pmm_set_beacon_location= NULL;
bool mmSetBeaconLocation(uint8_t address, int32_t x_mm, int32_t y_mm, int32_t z_mm) {
    if (pmm_set_beacon_location == NULL)
        return false;

    uint8_t buf[64];

    *((int32_t *) &buf[0])= x_mm;
    *((int32_t *) &buf[4])= y_mm;
    *((int32_t *) &buf[8])= z_mm;

    return (*pmm_set_beacon_location)(address, (void *) &buf[0]);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_beacons_distance) (void *pdata);
static pt_mm_set_beacons_distance pmm_set_beacons_distance= NULL;
bool mmSetBeaconsDistance(uint8_t address_1, uint8_t address_2, int32_t distance_mm) {
    if (pmm_set_beacons_distance == NULL)
        return false;

    uint8_t buf[64];

    buf[0]= address_1;
    buf[1]= address_2;
    *((int32_t *) &buf[2])= distance_mm;

    return (*pmm_set_beacons_distance)((void *) &buf[0]);
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_hedge_height) (uint8_t address, void *pdata);
static pt_mm_get_hedge_height pmm_get_hedge_height= NULL;
bool mmGetHedgeHeight(uint8_t address, int32_t *pheight_mm) {
    if (pmm_get_hedge_height == NULL)
        return false;

    uint8_t buf[64];

    bool res= (*pmm_get_hedge_height)(address, (void *) &buf[0]);
    if (res) {
      *pheight_mm= *((int32_t *) &buf[0]);
    }

    return res;
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_hedge_height) (uint8_t address, void *pdata);
static pt_mm_set_hedge_height pmm_set_hedge_height= NULL;
bool mmSetHedgeHeight(uint8_t address, int32_t height_mm) {
    if (pmm_set_hedge_height == NULL)
        return false;

    uint8_t buf[64];

    *((int32_t *) &buf[0])= height_mm;

    return (*pmm_set_hedge_height)(address, (void *) &buf[0]);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_beacon_height) (uint8_t address, void *pdata);
static pt_mm_get_beacon_height pmm_get_beacon_height= NULL;
bool mmGetBeaconHeight(uint8_t address, uint8_t submapId, int32_t *pheight_mm) {
    if (pmm_get_beacon_height == NULL)
        return false;

    uint8_t buf[64];

    buf[0]= submapId;
    bool res= (*pmm_get_beacon_height)(address, (void *) &buf[0]);
    if (res) {
      *pheight_mm= *((int32_t *) &buf[1]);
    }

    return res;
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_beacon_height) (uint8_t address, void *pdata);
static pt_mm_set_beacon_height pmm_set_beacon_height= NULL;
bool mmSetBeaconHeight(uint8_t address, uint8_t submapId, int32_t height_mm) {
    if (pmm_set_beacon_height == NULL)
        return false;

    uint8_t buf[64];

    buf[0]= submapId;
    *((int32_t *) &buf[1])= height_mm;

    return (*pmm_set_beacon_height)(address, (void *) &buf[0]);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_realtime_player_settings) (uint8_t address, void *pdata);
static pt_mm_get_realtime_player_settings pmm_get_realtime_player_settings= NULL;
bool mmGetRealtimePlayerSettings(uint8_t address, MarvelmindRealtimePlayerSettings *rtp) {
    if (pmm_get_realtime_player_settings == NULL)
        return false;

    uint8_t buf[64];

    bool res= (*pmm_get_realtime_player_settings)(address, (void *) &buf[0]);
    if (res) {
      rtp->rtpEnabled= (bool) buf[0];
      rtp->rtpForward= buf[1];
      rtp->rtpBackward= buf[2];
      rtp->reserved0= buf[3];
      rtp->reserved1= buf[4];
    }

    return res;
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_realtime_player_settings) (uint8_t address, void *pdata);
static pt_mm_set_realtime_player_settings pmm_set_realtime_player_settings= NULL;
bool mmSetRealtimePlayerSettings(uint8_t address, MarvelmindRealtimePlayerSettings *rtp) {
    if (pmm_set_realtime_player_settings == NULL)
        return false;

    uint8_t buf[64];

    buf[0]= rtp->rtpEnabled;
    buf[1]= rtp->rtpForward;
    buf[2]= rtp->rtpBackward;
    buf[3]= rtp->reserved0;
    buf[4]= rtp->reserved1;

    return (*pmm_set_realtime_player_settings)(address, (void *) &buf[0]);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_georeferencing_settings) (void *pdata);
static pt_mm_get_georeferencing_settings pmm_get_georeferencing_settings= NULL;
bool mmGetGeoreferencingSettings(MarvelmindGeoreferencingSettings *gr) {
    if (pmm_get_georeferencing_settings == NULL)
        return false;

    uint8_t buf[64];

    bool res= (*pmm_get_georeferencing_settings)((void *) &buf[0]);
    if (res) {
      gr->latitude_x100ndeg= *((int32_t *) &buf[0]);
      gr->longitude_x100ndeg= *((int32_t *) &buf[4]);
    }

    return res;
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_georeferencing_settings) (void *pdata);
static pt_mm_set_georeferencing_settings pmm_set_georeferencing_settings= NULL;
bool mmSetGeoreferencingSettings(MarvelmindGeoreferencingSettings *gr) {
    if (pmm_set_georeferencing_settings == NULL)
        return false;

    uint8_t buf[64];

    *((int32_t *) &buf[0])= gr->latitude_x100ndeg;
    *((int32_t *) &buf[4])= gr->longitude_x100ndeg;

    return (*pmm_set_georeferencing_settings)((void *) &buf[0]);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_update_positions_mode) (void *pdata);
static pt_mm_get_update_positions_mode pmm_get_update_positions_mode= NULL;
bool mmGetUpdatePositionsMode(MarvelmindUpdatePositionsMode *ups) {
    if (pmm_get_update_positions_mode == NULL)
        return false;

    uint8_t buf[64];
    uint8_t i;

    bool res= (*pmm_get_update_positions_mode)((void *) &buf[0]);
    if (res) {
      ups->mode= buf[0];
      for(i=0;i<7;i++)
        ups->reserved[i]= buf[1+i];
    }

    return res;
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_update_positions_mode) (void *pdata);
static pt_mm_set_update_positions_mode pmm_set_update_positions_mode= NULL;
bool mmSetUpdatePositionsMode(MarvelmindUpdatePositionsMode *ups) {
    if (pmm_set_update_positions_mode == NULL)
        return false;

    uint8_t buf[64];
    uint8_t i;

    buf[0]= ups->mode;
    for(i=0;i<7;i++)
        buf[1+i]= ups->reserved[i];

    return (*pmm_set_update_positions_mode)((void *) &buf[0]);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_update_positions_command) (void *pdata);
static pt_mm_set_update_positions_command pmm_set_update_positions_command= NULL;
bool mmSendUpdatePositionsCommand(MarvelmindUpdatePositionsCommand *upc) {
    if (pmm_set_update_positions_command == NULL)
        return false;

    uint8_t buf[64];
    uint8_t i;

    for(i=0;i<8;i++)
        buf[i]= upc->reserved[i];

    return (*pmm_set_update_positions_command)((void *) &buf[0]);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_alarm_state) (uint8_t address, void *pdata);
static pt_mm_set_alarm_state pmm_set_alarm_state_command= NULL;
bool mmSendAlarmState(uint8_t address, MarvelmindAlarmState *mas) {
    if (pmm_set_alarm_state_command == NULL)
        return false;

    uint8_t buf[64];
    uint8_t i;

    buf[0]= mas->alarmMode;
    buf[1]= mas->zoneIndex;
    for(i=0;i<6;i++)
        buf[i+2]= mas->reserved[i];

    return (*pmm_set_alarm_state_command)(address, (void *) &buf[0]);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_send_user_payload) (uint8_t address, void *pdata);
static pt_mm_send_user_payload pmm_pt_send_user_payload= NULL;
bool mmSendUserPayload(uint8_t address, MarvelmindSendUserPayload *mup) {
    if (pmm_pt_send_user_payload == NULL)
        return false;

    uint8_t buf[300];
    uint16_t i;

    buf[0]= mup->dsize;
    for(i=0;i<mup->dsize;i++)
        buf[i+1]= mup->data[i];

    return (*pmm_pt_send_user_payload)(address, (void *) &buf[0]);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_user_payload) (void *pdata);
static pt_mm_get_user_payload pmm_pt_get_user_payload= NULL;
bool mmGetUserPayload(MarvelmindGetUserPayload *mup) {
    if (pmm_pt_get_user_payload == NULL)
        return false;

    uint8_t buf[300];
    uint16_t i;

    bool res= (*pmm_pt_get_user_payload)((void *) &buf[0]);

    if (res) {
      mup->address= buf[0];
      memcpy(&mup->timestamp, &buf[1], 8);
      mup->dsize= buf[9];
      for(i=0;i<mup->dsize;i++)
        mup->data[i]= buf[10+i];
    }

    return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_motors_control) (uint8_t address, void *pdata);
static pt_mm_set_motors_control pmm_set_motors_control= NULL;
bool mmSetMotorsControl(uint8_t address, MarvelmindMotorsSettings *motorsCtrl) {
    if (pmm_set_motors_control == NULL)
        return false;

    uint8_t buf[64];

    buf[0]= motorsCtrl->mode;
    buf[1]= motorsCtrl->moveType;
    buf[2]= motorsCtrl->levelPercents;

    return (*pmm_set_motors_control)(address, (void *) &buf[0]);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_robot_program_item) (uint8_t address, void *pdata);
static pt_mm_set_robot_program_item pmm_set_robot_program_item= NULL;
bool mmSetRobotProgramItem(uint8_t address, MarvelmindRobotProgramItem *progItem) {
    if (pmm_set_robot_program_item == NULL)
        return false;

    uint8_t buf[64];

    buf[0]= progItem->itemIndex;
    buf[1]= progItem->totalItemsNum;

    buf[2]= progItem->opCode;
    *((int16_t *) &buf[3])= progItem->param1;
    *((int16_t *) &buf[5])= progItem->param2;
    *((int16_t *) &buf[7])= progItem->param3;

    return (*pmm_set_robot_program_item)(address, (void *) &buf[0]);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_robot_command) (uint8_t address, void *pdata);
static pt_mm_set_robot_command pmm_set_robot_command= NULL;
bool mmSetRobotCommand(uint8_t address, MarvelmindRobotCommand *robotCommand) {
    if (pmm_set_robot_command == NULL)
        return false;

    uint8_t buf[64];

    buf[0]= robotCommand->commandId;

    *((int16_t *) &buf[1])= robotCommand->param1;
    *((int16_t *) &buf[3])= robotCommand->param2;
    *((int16_t *) &buf[5])= robotCommand->param3;

    return (*pmm_set_robot_command)(address, (void *) &buf[0]);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_robot_position) (uint8_t address, void *pdata);
static pt_mm_set_robot_position pmm_set_robot_position= NULL;
bool mmSetRobotPosition(uint8_t address, MarvelmindRobotPosition *robotPosition) {
    if (pmm_set_robot_position == NULL)
        return false;

    uint8_t buf[64];
    uint8_t i;

    *((int32_t *) &buf[0])= robotPosition->x_mm;
    *((int32_t *) &buf[4])= robotPosition->y_mm;
    *((int32_t *) &buf[8])= robotPosition->z_mm;

    *((uint16_t *) &buf[12])= robotPosition->angle;

    for(i=0;i<18;i++) {
       buf[14+i]= robotPosition->reserved[i];
    }

    return (*pmm_set_robot_position)(address, (void *) &buf[0]);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_robot_telemetry)(uint8_t address, void *pdata);
static pt_mm_get_robot_telemetry pmm_get_robot_telemetry= NULL;
bool mmGetRobotTelemetry(uint8_t address, MarvelmindRobotTelemetry *telemetry) {
  if (pmm_get_robot_telemetry == NULL)
    return false;

  uint8_t buf[128];
  uint8_t i;
  bool res= (*pmm_get_robot_telemetry)(address, &buf[0]);

  for(i=0;i<64;i++) {
     telemetry->data[i]= buf[i];
  }

  return res;
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_robot_settings)(uint8_t address, void *pdata);
static pt_mm_get_robot_settings pmm_get_robot_settings= NULL;
bool mmGetRobotSettings(uint8_t address, MarvelmindRobotSettings *robotSettings) {
  if (pmm_get_robot_settings == NULL)
    return false;

  uint8_t buf[128];
  uint8_t i;

  buf[0]= robotSettings->page;
  bool res= (*pmm_get_robot_settings)(address, &buf[0]);

  for(i=0;i<32;i++) {
     robotSettings->data[i]= buf[i+2];
  }

  return res;
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_robot_settings)(uint8_t address, void *pdata);
static pt_mm_set_robot_settings pmm_set_robot_settings= NULL;
bool mmSetRobotSettings(uint8_t address, MarvelmindRobotSettings *robotSettings) {
  if (pmm_set_robot_settings == NULL)
    return false;

  uint8_t buf[64];
  uint8_t i;

  buf[0]= robotSettings->page;
  buf[1]= 32;

  for(i=0;i<32;i++) {
       buf[2+i]= robotSettings->data[i];
  }

  return (*pmm_set_robot_settings)(address, (void *) &buf[0]);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_robot_v100_power)(void *pdata);
static pt_mm_get_robot_v100_power pmm_get_robot_v100_power= NULL;
bool mmRobotV100GetPower(MarvelmindRobotV100Power *power) {
  if (pmm_get_robot_v100_power == NULL)
    return false;

  uint8_t buf[128];
  uint8_t i;
  bool res= (*pmm_get_robot_v100_power)(&buf[0]);

  power->batteryVoltage_x10mv= *((uint16_t *) &buf[0]);
  power->totalCurrent_x10ma= *((uint16_t *) &buf[2]);
  power->motorsCurrent_x10ma= *((uint16_t *) &buf[4]);
  power->batteryCapacity_per= *((uint8_t *) &buf[6]);
  power->timestamp_ms= *((uint32_t *) &buf[7]);
  power->flags= *((uint8_t *) &buf[11]);

  for(i=0;i<20;i++) {
     power->reserved[i]= buf[12+i];
  }

  return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_robot_v100_encoders)(void *pdata);
static pt_mm_get_robot_v100_encoders pmm_get_robot_v100_encoders= NULL;
bool mmRobotV100GetEncoders(MarvelmindRobotV100Encoders *encoders) {
  if (pmm_get_robot_v100_encoders == NULL)
    return false;

  uint8_t buf[128];
  uint8_t i;
  bool res= (*pmm_get_robot_v100_encoders)(&buf[0]);

  encoders->leftEncoderPath_cm= *((int32_t *) &buf[0]);
  encoders->rightEncoderPath_cm= *((int32_t *) &buf[4]);
  encoders->timestamp_ms= *((uint32_t *) &buf[8]);

  for(i=0;i<20;i++) {
     encoders->reserved[i]= buf[12+i];
  }

  return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_robot_v100_lidars)(void *pdata);
static pt_mm_get_robot_v100_lidars pmm_get_robot_v100_lidars= NULL;
bool mmRobotV100GetLidars(MarvelmindRobotV100Lidars *lidars) {
  if (pmm_get_robot_v100_lidars == NULL)
    return false;

  uint8_t buf[128];
  uint8_t i;
  bool res= (*pmm_get_robot_v100_lidars)(&buf[0]);

  uint8_t ofs= 0;
  for(i=0;i<RV100_LIDARS_NUM;i++) {
    uint16_t v= *((int32_t *) &buf[ofs]);

    lidars->lidars[i].range_mm= v&0x0fff;
    lidars->lidars[i].status= (v>>12)&0x0f;

    ofs+= 2;
  }

  lidars->timestamp_ms= *((uint32_t *) &buf[28]);

  for(i=0;i<32;i++) {
     lidars->reserved[i]= buf[32+i];
  }

  return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_robot_v100_location)(void *pdata);
static pt_mm_get_robot_v100_location pmm_get_robot_v100_location= NULL;
bool mmRobotV100GetLocation(MarvelmindRobotV100Location *pos) {
  if (pmm_get_robot_v100_location == NULL)
    return false;

  uint8_t buf[128];
  uint8_t i;
  bool res= (*pmm_get_robot_v100_location)(&buf[0]);

  pos->x_m=  ((float) (*((int32_t *) &buf[0])))/1000.0f;
  pos->y_m=  ((float) (*((int32_t *) &buf[4])))/1000.0f;
  pos->z_m=  ((float) (*((int32_t *) &buf[8])))/1000.0f;

  pos->yaw_angle_deg=  ((float) (*((uint16_t *) &buf[12])))/10.0f;

  pos->flags= buf[14];

  pos->timestamp_ms= *((uint32_t *) &buf[15]);

  for(i=0;i<13;i++) {
     pos->reserved[i]= buf[19+i];
  }

  return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_get_robot_v100_raw_imu)(void *pdata);
static pt_mm_get_robot_v100_raw_imu pmm_get_robot_v100_raw_imu= NULL;
bool mmRobotV100GetRawIMU(MarvelmindRobotV100RawIMU *rawIMU) {
  if (pmm_get_robot_v100_raw_imu == NULL)
    return false;

  uint8_t buf[128];
  uint8_t i;
  bool res= (*pmm_get_robot_v100_raw_imu)(&buf[0]);

  rawIMU->ax_mg=  (*((int16_t *) &buf[0]));
  rawIMU->ay_mg=  (*((int16_t *) &buf[2]));
  rawIMU->az_mg=  (*((int16_t *) &buf[4]));

  rawIMU->gx=  (*((int16_t *) &buf[6]));
  rawIMU->gy=  (*((int16_t *) &buf[8]));
  rawIMU->gz=  (*((int16_t *) &buf[10]));

  rawIMU->timestamp_ms= *((uint32_t *) &buf[12]);

  for(i=0;i<16;i++) {
     rawIMU->reserved[i]= buf[16+i];
  }

  return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_set_robot_v100_motors)(void *pdata);
static pt_mm_set_robot_v100_motors pmm_set_robot_v100_motors= NULL;
bool mmRobotV100SetMotors(MarvelmindRobotV100Motors *motors) {
  if (pmm_set_robot_v100_motors == NULL)
    return false;

  uint8_t buf[64];
  uint8_t i;

  *((uint8_t *) &buf[0])= motors->motorsMode;
  *((uint8_t *) &buf[1])= motors->leftMotorSpeed;
  *((uint8_t *) &buf[2])= motors->rightMotorSpeed;
  *((uint8_t *) &buf[3])= motors->leftMotorFlags;
  *((uint8_t *) &buf[4])= motors->rightMotorFlags;

  for(i=0;i<11;i++) {
    *((uint8_t *) &buf[5+i])= motors->reserved[i];
  }

  bool res= (*pmm_set_robot_v100_motors)((void *) &buf[0]);

  return res;
}

//////

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_device_is_modem)(uint8_t deviceType);
static pt_mm_device_is_modem pmm_device_is_modem= NULL;
bool mmDeviceIsModem(uint8_t deviceType) {
  if (pmm_device_is_modem == NULL)
    return false;

  return (*pmm_device_is_modem)(deviceType);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_device_is_beacon)(uint8_t deviceType);
static pt_mm_device_is_beacon pmm_device_is_beacon= NULL;
bool mmDeviceIsBeacon(uint8_t deviceType) {
  if (pmm_device_is_beacon == NULL)
    return false;

  return (*pmm_device_is_beacon)(deviceType);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_device_is_hedgehog) (uint8_t deviceType);
static pt_mm_device_is_hedgehog pmm_device_is_hedgehog= NULL;
bool mmDeviceIsHedgehog(uint8_t deviceType) {
  if (pmm_device_is_hedgehog == NULL)
    return false;

  return (*pmm_device_is_hedgehog)(deviceType);
}

typedef bool MM_STDCALL1 (MM_STDCALL2 *pt_mm_device_is_robot) (uint8_t deviceType);
static pt_mm_device_is_robot pmm_device_is_robot= NULL;
bool mmDeviceIsRobot(uint8_t deviceType) {
  if (pmm_device_is_robot == NULL)
    return false;

  return (*pmm_device_is_robot)(deviceType);
}

//////////////////////////////////////////////////////////////////

#define MM_MAX_REQUEST_SIZE 1024
typedef bool MM_STDCALL1(MM_STDCALL2* pt_simple_api_call)(void* pdata);
static bool marvelmindAPICall_simple(pt_simple_api_call cfunc, uint8_t* request, uint32_t request_size, uint8_t* response, uint32_t* response_size, uint32_t const_response_size) {
    if (cfunc == NULL) return false;

    uint8_t buf[MM_MAX_REQUEST_SIZE];
    if (request_size > 0) {
        memcpy(&buf[0], request, request_size);
    }

    bool result= cfunc(&buf[0]);

    *response_size = const_response_size;
    if (const_response_size != 0) {
        memcpy(response, &buf[0], const_response_size);
    }

    return result;
}

typedef bool MM_STDCALL1(MM_STDCALL2* pt_simple_api_call_addressed)(uint8_t address, void* pdata);
static bool marvelmindAPICall_simple_addressed(pt_simple_api_call_addressed cfunc, uint8_t* request, uint32_t request_size, uint8_t* response, uint32_t* response_size, uint32_t const_response_size) {
    if (cfunc == NULL) return false;

    uint8_t buf[MM_MAX_REQUEST_SIZE];
    if (request_size > 0) {
        memcpy(&buf[0], request, request_size);
    }

    bool result = cfunc(buf[0], &buf[1]);

    *response_size = const_response_size;
    if (const_response_size != 0) {
        memcpy(response, &buf[1], const_response_size);
    }

    return result;
}

typedef bool MM_STDCALL1(MM_STDCALL2* pt_simple_api_call_send1byte)(uint8_t v8);
static bool marvelmindAPICall_simple_send1byte(pt_simple_api_call_send1byte cfunc, uint8_t* request, uint32_t request_size, uint8_t* response, uint32_t* response_size) {
    if (cfunc == NULL) return false;

    if (request_size == 0)
        return false;

    bool result = cfunc(request[0]);

    *response_size = 0;
    response[0]= 0;

    return result;
}

static bool marvelmindAPICall_devices_list(pt_simple_api_call cfunc, uint8_t* request, uint32_t request_size, uint8_t* response, uint32_t* response_size) {
    if (cfunc == NULL) return false;

    uint8_t buf[MM_MAX_REQUEST_SIZE];
    if (request_size > 0) {
        memcpy(&buf[0], request, request_size);
    }

    bool result = cfunc(&buf[0]);

    *response_size = 1+buf[0]*9;
    if (*response_size != 0) {
        memcpy(response, &buf[0], *response_size);
    }

    return result;
}

static bool marvelmindAPICall_locations(pt_simple_api_call cfunc, uint8_t* request, uint32_t request_size, uint8_t* response, uint32_t* response_size) {
    if (cfunc == NULL) return false;

    uint8_t buf[MM_MAX_REQUEST_SIZE];
    if (request_size > 0) {
        memcpy(&buf[0], request, request_size);
    }

    bool result = cfunc(&buf[0]);

    *response_size = MM_LOCATIONS_PACK_SIZE*20 + 7 + buf[MM_LOCATIONS_PACK_SIZE * 20 + 6];
    if (*response_size != 0) {
        memcpy(response, &buf[0], *response_size);
    }

    return result;
}

static bool marvelmindAPICall_distances(pt_simple_api_call cfunc, uint8_t* request, uint32_t request_size, uint8_t* response, uint32_t* response_size) {
    if (cfunc == NULL) return false;

    uint8_t buf[MM_MAX_REQUEST_SIZE];
    if (request_size > 0) {
        memcpy(&buf[0], request, request_size);
    }

    bool result = cfunc(&buf[0]);

    *response_size = 1 + buf[0] * 9;
    if (*response_size != 0) {
        memcpy(response, &buf[0], *response_size);
    }

    return result;
}

static bool marvelmindAPICall_read_flash_dump(uint8_t* request, uint32_t request_size, uint8_t* response, uint32_t* response_size) {
    if (pmm_read_flash_dump == NULL) return false;

    uint8_t buf[MM_MAX_REQUEST_SIZE];
    if (request_size < 8) {
        return false;
    }

    uint32_t offset= *((uint32_t*)&request[0]);
    uint32_t size = *((uint32_t*)&request[4]);

    bool result = pmm_read_flash_dump(offset , size,  &buf[8]);

    *response_size = size;
    if (*response_size != 0) {
        memcpy(response, &buf[8], *response_size);
    }

    return result;
}

static bool marvelmindAPICall_write_flash_dump(uint8_t* request, uint32_t request_size, uint8_t* response, uint32_t* response_size) {
    if (pmm_write_flash_dump == NULL) return false;

    uint8_t buf[MM_MAX_REQUEST_SIZE];
    if (request_size < 8) {
        return false;
    }

    memcpy(&buf[0], request, request_size);

    uint32_t offset = *((uint32_t*)&request[0]);
    uint32_t size = *((uint32_t*)&request[4]);

    bool result = pmm_write_flash_dump(offset, size, &buf[0]);

    *response_size = 0;
    response[0]= 0;

    return result;
}

static bool marvelmindAPICall_get_user_payload(uint8_t* request, uint32_t request_size, uint8_t* response, uint32_t* response_size) {
    if (pmm_pt_get_user_payload == NULL) return false;

    uint8_t buf[MM_MAX_REQUEST_SIZE];
    if (request_size > 0) {
        memcpy(&buf[0], request, request_size);
    }

    bool result = pmm_pt_get_user_payload(&buf[0]);

    *response_size =10 + buf[9];
    if (*response_size != 0) {
        memcpy(response, &buf[0], *response_size);
    }

    return result;
}

static bool marvelmindAPICall_close_port(uint32_t* response_size) {
	if (pmm_close_port == NULL) return false;
	
	pmm_close_port();
	*response_size= 0;
	
	return true;
}

typedef bool MM_STDCALL1(MM_STDCALL2* pt_simple_api_call_no_request)(void);
static bool marvelmindAPICall_simple_no_request(pt_simple_api_call_no_request cfunc,  uint8_t* response, uint32_t* response_size, uint32_t const_response_size) {
    if (cfunc == NULL) return false;

    uint8_t buf[MM_MAX_REQUEST_SIZE];

    bool result = cfunc();

    *response_size = const_response_size;
    if (const_response_size != 0) {
        memcpy(response, &buf[0], const_response_size);
    }

    return result;
}


bool marvelmindAPICall(int64_t command_id, uint8_t* request, uint32_t request_size, uint8_t* response, uint32_t* response_size, int32_t* error_code) {
    bool result = false;

    if (request_size > MM_MAX_REQUEST_SIZE) {
        *error_code =  MM_API_ERROR_REQUEST_TOO_LARGE;
        return false;
    }

    switch (command_id) {
    case MM_API_ID_API_VERSION: {
        result = marvelmindAPICall_simple(pmm_api_version, request, request_size, response, response_size, sizeof(uint32_t));
        break;
    }

    case MM_API_ID_GET_LAST_ERROR: {
        result = marvelmindAPICall_simple(pmm_get_last_error, request, request_size, response, response_size, sizeof(uint32_t));
        break;
    }
 
    case MM_API_ID_OPEN_PORT: {
        result = marvelmindAPICall_simple_no_request(pmm_open_port, response, response_size, 0);
        break;
    }

    case MM_API_ID_OPEN_PORT_BY_NAME: {
        result = marvelmindAPICall_simple(pmm_open_port_by_name, request, request_size, response, response_size, 0);
        break;
    }

    case MM_API_ID_OPEN_PORT_UDP: {
        result = marvelmindAPICall_simple(pmm_open_port_udp, request, request_size, response, response_size, 0);
        break;
    }

    case MM_API_ID_CLOSE_PORT: {
        result = marvelmindAPICall_close_port(response_size);
        break;
    }

    case MM_API_ID_GET_VERSION_AND_ID: {
        result = marvelmindAPICall_simple_addressed(pmm_get_version_and_id, request, request_size, response, response_size, 9);
        //if (!result) {
        //    result = marvelmindAPICall_simple_addressed(pmm_get_version_and_id, request, request_size, response, response_size, 9);
        //}
        break;
    }

    case MM_API_ID_GET_DEVICES_LIST: {
        result = marvelmindAPICall_devices_list(pmm_get_devices_list, request, request_size, response, response_size);
        //if (!result) {
        //    result = marvelmindAPICall_simple_addressed(pmm_get_version_and_id, request, request_size, response, response_size, 9);
        //}
        break;
    }

    case MM_API_ID_WAKE_DEVICE: {
        result = marvelmindAPICall_simple_send1byte(pmm_wake_device, request, request_size, response, response_size);
        break;
    }

    case MM_API_ID_SLEEP_DEVICE: {
        result = marvelmindAPICall_simple_send1byte(pmm_sleep_device, request, request_size, response, response_size);
        break;
    }

    case MM_API_ID_GET_BEACON_TELEMETRY: {
        result = marvelmindAPICall_simple_addressed(pmm_get_beacon_tele, request, request_size, response, response_size, 24);
        break;
    }

    case MM_API_ID_GET_LAST_LOCATIONS2: {
        result = marvelmindAPICall_locations(pmm_get_last_locations2, request, request_size, response, response_size);
        break;
    }

    case MM_API_ID_GET_LAST_DISTANCES: {
        result = marvelmindAPICall_distances(pmm_get_last_distances, request, request_size, response, response_size);
        break;
    }

    case MM_API_ID_GET_UPDATE_RATE: {
        result = marvelmindAPICall_simple(pmm_get_update_rate_setting, request, request_size, response, response_size, 4);
        break;
    }

    case MM_API_ID_SET_UPDATE_RATE: {
        result = marvelmindAPICall_simple(pmm_set_update_rate_setting, request, request_size, response, response_size, 0);
        break;
    }

    case MM_API_ID_ADD_SUBMAP: {
        result = marvelmindAPICall_simple_send1byte(pmm_add_submap, request, request_size, response, response_size);
        break;
    }

    case MM_API_ID_DELETE_SUBMAP: {
        result = marvelmindAPICall_simple_send1byte(pmm_delete_submap, request, request_size, response, response_size);
        break;
    }

    case MM_API_ID_FREEZE_SUBMAP: {
        result = marvelmindAPICall_simple_send1byte(pmm_freeze_submap, request, request_size, response, response_size);
        break;
    }

    case MM_API_ID_UNFREEZE_SUBMAP: {
        result = marvelmindAPICall_simple_send1byte(pmm_unfreeze_submap, request, request_size, response, response_size);
        break;
    }

    case MM_API_ID_ERASE_MAP: {
        result = marvelmindAPICall_simple_no_request(pmm_erase_map, response, response_size, 0);
        break;
    }

    case MM_API_ID_SET_DEFAULT: {
        result = marvelmindAPICall_simple_send1byte(pmm_set_default_settings, request, request_size, response, response_size);
        break;
    }

    case MM_API_ID_FREEZE_MAP: {
        result = marvelmindAPICall_simple_no_request(pmm_freeze_map, response, response_size, 0);
        break;
    }

    case MM_API_ID_UNFREEZE_MAP: {
        result = marvelmindAPICall_simple_no_request(pmm_unfreeze_map, response, response_size, 0);
        break;
    }

    case MM_API_ID_BEACONS_TO_AXES: {
        result = marvelmindAPICall_simple(pmm_beacons_to_axes, request, request_size, response, response_size, 0);
        break;
    }

    case MM_API_ID_READ_FLASH_DUMP: {
        result = marvelmindAPICall_read_flash_dump(request, request_size, response, response_size);
        break;
    }

    case MM_API_ID_WRITE_FLASH_DUMP: {
        result = marvelmindAPICall_write_flash_dump(request, request_size, response, response_size);
        break;
    }

    case MM_API_ID_RESET_DEVICE: {
        result = marvelmindAPICall_simple_send1byte(pmm_reset_device, request, request_size, response, response_size);
        break;
    }

    case MM_API_ID_GET_AIR_TEMPERATURE: {
        result = marvelmindAPICall_simple(pmm_get_air_temperature, request, request_size, response, response_size, 1);
        break;
    }

    case MM_API_ID_SET_AIR_TEMPERATURE: {
        result = marvelmindAPICall_simple(pmm_set_air_temperature, request, request_size, response, response_size, 0);
        break;
    }

    case MM_API_ID_SET_BEACON_LOCATION: {
        result = marvelmindAPICall_simple_addressed(pmm_set_beacon_location, request, request_size, response, response_size, 0);
        break;
    }

    case MM_API_ID_SET_BEACONS_DISTANCE: {
        result = marvelmindAPICall_simple(pmm_set_beacons_distance, request, request_size, response, response_size, 0);
        break;
    }

    case MM_API_ID_GET_HEDGE_HEIGHT: {
        result = marvelmindAPICall_simple_addressed(pmm_get_hedge_height, request, request_size, response, response_size, 4);
        break;
    }

    case MM_API_ID_SET_HEDGE_HEIGHT: {
        result = marvelmindAPICall_simple_addressed(pmm_set_hedge_height, request, request_size, response, response_size, 0);
        break;
    }
 
    case MM_API_ID_GET_BEACON_HEIGHT: {
        result = marvelmindAPICall_simple_addressed(pmm_get_beacon_height, request, request_size, response, response_size, 5);
        break;
    }

    case MM_API_ID_SET_BEACON_HEIGHT: {
        result = marvelmindAPICall_simple_addressed(pmm_set_beacon_height, request, request_size, response, response_size, 0);
        break;
    }

    case MM_API_ID_GET_RTP_SETTINGS: {
        result = marvelmindAPICall_simple_addressed(pmm_get_realtime_player_settings, request, request_size, response, response_size, 5);
        break;
    }

    case MM_API_ID_SET_RTP_SETTINGS: {
        result = marvelmindAPICall_simple_addressed(pmm_set_realtime_player_settings, request, request_size, response, response_size, 0);
        break;
    }

    case MM_API_ID_GET_GEOREFERENCING: {
        result = marvelmindAPICall_simple(pmm_get_georeferencing_settings, request, request_size, response, response_size, 8);
        break;
    }

    case MM_API_ID_SET_GEOREFERENCING: {
        result = marvelmindAPICall_simple(pmm_set_georeferencing_settings, request, request_size, response, response_size, 0);
        break;
    }

    case MM_API_ID_GET_UPDATE_POSITIONS_MODE: {
        result = marvelmindAPICall_simple(pmm_get_update_positions_mode, request, request_size, response, response_size, 8);
        break;
    }

    case MM_API_ID_SET_UPDATE_POSITIONS_MODE: {
        result = marvelmindAPICall_simple(pmm_set_update_positions_mode, request, request_size, response, response_size, 0);
        break;
    }

    case MM_API_ID_SEND_UPDATE_POSITIONS_COMMAND: {
        result = marvelmindAPICall_simple(pmm_set_update_positions_command, request, request_size, response, response_size, 0);
        break;
    }

    case MM_API_ID_SET_ALARM_STATE: {
        result = marvelmindAPICall_simple_addressed(pmm_set_alarm_state_command, request, request_size, response, response_size, 0);
        break;
    }

    case MM_API_ID_GET_USER_PAYLOAD: {
        result = marvelmindAPICall_get_user_payload(request, request_size, response, response_size);
        break;
    }

    case MM_API_ID_SET_USER_PAYLOAD: {
        result = marvelmindAPICall_simple_addressed(pmm_pt_send_user_payload, request, request_size, response, response_size, 0);
        break;
    }

    case MM_API_ID_DEVICE_IS_MODEM: {
        result = marvelmindAPICall_simple_send1byte(pmm_device_is_modem, request, request_size, response, response_size);
        break;
    }

    case MM_API_ID_DEVICE_IS_BEACON: {
        result = marvelmindAPICall_simple_send1byte(pmm_device_is_beacon, request, request_size, response, response_size);
        break;
    }

    case MM_API_ID_DEVICE_IS_HEDGEHOG: {
        result = marvelmindAPICall_simple_send1byte(pmm_device_is_hedgehog, request, request_size, response, response_size);
        break;
    }

    case MM_API_ID_DEVICE_IS_ROBOT: {
        result = marvelmindAPICall_simple_send1byte(pmm_device_is_robot, request, request_size, response, response_size);
        break;
    }

    }// switch (command_id)

    if (!result) {
		uint32_t errRes;
        mmGetLastError(&errRes);
        *error_code = (int32_t) errRes;
    }
    else {
        *error_code = 0;
    }

    return result;
}// marvelmindAPICall

//////

void marvelmindAPILoad(char *dll_filepath) {
#ifdef WIN32
  if (dll_filepath == NULL) {
    mmLibrary= LoadLibrary("dashapi.dll");
  } else {
      mmLibrary= LoadLibrary(dll_filepath);
  }

  pmm_api_version= (pt_mm_api_version ) GetProcAddress(mmLibrary, "mm_api_version");
  pmm_get_last_error= (pt_mm_get_last_error ) GetProcAddress(mmLibrary, "mm_get_last_error");

  pmm_open_port= (pt_mm_open_port ) GetProcAddress(mmLibrary, "mm_open_port");
  pmm_open_port_by_name= (pt_mm_open_port_by_name ) GetProcAddress(mmLibrary, "mm_open_port_by_name");
  pmm_open_port_udp= (pt_mm_open_port_udp ) GetProcAddress(mmLibrary, "mm_open_port_udp");
  pmm_close_port= (pt_mm_close_port ) GetProcAddress(mmLibrary, "mm_close_port");

  pmm_get_version_and_id= (pt_mm_get_version_and_id ) GetProcAddress(mmLibrary, "mm_get_device_version_and_id");
  pmm_get_devices_list= (pt_mm_get_devices_list ) GetProcAddress(mmLibrary, "mm_get_devices_list");

  pmm_wake_device= (pt_mm_wake_device ) GetProcAddress(mmLibrary, "mm_wake_device");
  pmm_sleep_device= (pt_mm_sleep_device ) GetProcAddress(mmLibrary, "mm_send_to_sleep_device");

  pmm_get_beacon_tele= (pt_mm_get_beacon_tele ) GetProcAddress(mmLibrary, "mm_get_beacon_telemetry");

  pmm_get_last_locations= (pt_mm_get_last_locations ) GetProcAddress(mmLibrary, "mm_get_last_locations");
  pmm_get_last_locations2= (pt_mm_get_last_locations2 ) GetProcAddress(mmLibrary, "mm_get_last_locations2");

  pmm_get_last_distances= (pt_mm_get_last_distances ) GetProcAddress(mmLibrary, "mm_get_last_distances");

  pmm_get_update_rate_setting= (pt_mm_get_update_rate_setting ) GetProcAddress(mmLibrary, "mm_get_update_rate_setting");
  pmm_set_update_rate_setting= (pt_mm_set_update_rate_setting ) GetProcAddress(mmLibrary, "mm_set_update_rate_setting");

  pmm_add_submap= (pt_mm_add_submap ) GetProcAddress(mmLibrary, "mm_add_submap");
  pmm_delete_submap= (pt_mm_delete_submap ) GetProcAddress(mmLibrary, "mm_delete_submap");

  pmm_freeze_submap= (pt_mm_freeze_submap ) GetProcAddress(mmLibrary, "mm_freeze_submap");
  pmm_unfreeze_submap= (pt_mm_unfreeze_submap ) GetProcAddress(mmLibrary, "mm_unfreeze_submap");

  pmm_get_submap_settings= (pt_mm_get_submap_settings ) GetProcAddress(mmLibrary, "mm_get_submap_settings");
  pmm_set_submap_settings= (pt_mm_set_submap_settings ) GetProcAddress(mmLibrary, "mm_set_submap_settings");

  pmm_get_ultrasound_settings= (pt_mm_get_ultrasound_settings ) GetProcAddress(mmLibrary, "mm_get_ultrasound_settings");
  pmm_set_ultrasound_settings= (pt_mm_set_ultrasound_settings ) GetProcAddress(mmLibrary, "mm_set_ultrasound_settings");

  pmm_erase_map= (pt_mm_erase_map ) GetProcAddress(mmLibrary, "mm_erase_map");
  pmm_set_default_settings= (pt_mm_set_default_settings ) GetProcAddress(mmLibrary, "mm_set_default_settings");

  pmm_freeze_map= (pt_mm_freeze_map ) GetProcAddress(mmLibrary, "mm_freeze_map");
  pmm_unfreeze_map= (pt_mm_unfreeze_map ) GetProcAddress(mmLibrary, "mm_unfreeze_map");

  pmm_beacons_to_axes= (pt_mm_beacons_to_axes ) GetProcAddress(mmLibrary, "mm_beacons_to_axes");

  pmm_read_flash_dump= (pt_read_flash_dump ) GetProcAddress(mmLibrary, "mm_read_flash_dump");
  pmm_write_flash_dump= (pt_write_flash_dump ) GetProcAddress(mmLibrary, "mm_write_flash_dump");

  pmm_reset_device= (pt_mm_reset_device ) GetProcAddress(mmLibrary, "mm_reset_device");

  pmm_get_air_temperature= (pt_mm_get_air_temperature ) GetProcAddress(mmLibrary, "mm_get_air_temperature");
  pmm_set_air_temperature= (pt_mm_set_air_temperature ) GetProcAddress(mmLibrary, "mm_set_air_temperature");

  pmm_set_beacon_location= (pt_mm_set_beacon_location ) GetProcAddress(mmLibrary, "mm_set_beacon_location");
  pmm_set_beacons_distance= (pt_mm_set_beacons_distance ) GetProcAddress(mmLibrary, "mm_set_beacons_distance");

  pmm_get_hedge_height= (pt_mm_get_hedge_height ) GetProcAddress(mmLibrary, "mm_get_hedge_height");
  pmm_set_hedge_height= (pt_mm_set_hedge_height ) GetProcAddress(mmLibrary, "mm_set_hedge_height");
  pmm_get_beacon_height= (pt_mm_get_beacon_height ) GetProcAddress(mmLibrary, "mm_get_beacon_height");
  pmm_set_beacon_height= (pt_mm_set_beacon_height ) GetProcAddress(mmLibrary, "mm_set_beacon_height");

  pmm_get_realtime_player_settings= (pt_mm_get_realtime_player_settings ) GetProcAddress(mmLibrary, "mm_get_realtime_player_settings");
  pmm_set_realtime_player_settings= (pt_mm_set_realtime_player_settings ) GetProcAddress(mmLibrary, "mm_set_realtime_player_settings");

  pmm_get_georeferencing_settings= (pt_mm_get_georeferencing_settings ) GetProcAddress(mmLibrary, "mm_get_georeferencing_settings");
  pmm_set_georeferencing_settings= (pt_mm_set_georeferencing_settings ) GetProcAddress(mmLibrary, "mm_set_georeferencing_settings");

  pmm_get_update_positions_mode= (pt_mm_get_update_positions_mode ) GetProcAddress(mmLibrary, "mm_get_update_position_mode");
  pmm_set_update_positions_mode= (pt_mm_set_update_positions_mode ) GetProcAddress(mmLibrary, "mm_set_update_position_mode");
  pmm_set_update_positions_command= (pt_mm_set_update_positions_command ) GetProcAddress(mmLibrary, "mm_set_update_position_command");

  pmm_set_alarm_state_command= (pt_mm_set_alarm_state ) GetProcAddress(mmLibrary, "mm_set_alarm_state");
  pmm_pt_send_user_payload= (pt_mm_send_user_payload ) GetProcAddress(mmLibrary, "mm_send_user_payload_data");
  pmm_pt_get_user_payload= (pt_mm_get_user_payload ) GetProcAddress(mmLibrary, "mm_get_user_payload_data");

  pmm_set_motors_control= (pt_mm_set_motors_control ) GetProcAddress(mmLibrary, "mm_set_robot_motors_control");
  pmm_set_robot_program_item= (pt_mm_set_robot_program_item ) GetProcAddress(mmLibrary, "mm_set_robot_program_item");
  pmm_set_robot_command= (pt_mm_set_robot_command ) GetProcAddress(mmLibrary, "mm_set_robot_command");
  pmm_set_robot_position= (pt_mm_set_robot_position ) GetProcAddress(mmLibrary, "mm_set_robot_position");
  pmm_get_robot_telemetry= (pt_mm_get_robot_telemetry ) GetProcAddress(mmLibrary, "mm_get_robot_telemetry");

  pmm_get_robot_settings= (pt_mm_get_robot_settings ) GetProcAddress(mmLibrary, "mm_get_robot_settings");
  pmm_set_robot_settings= (pt_mm_set_robot_settings ) GetProcAddress(mmLibrary, "mm_set_robot_settings");

  pmm_get_robot_v100_power= (pt_mm_get_robot_v100_power ) GetProcAddress(mmLibrary, "mm_robotv100_get_power");
  pmm_get_robot_v100_encoders= (pt_mm_get_robot_v100_encoders ) GetProcAddress(mmLibrary, "mm_robotv100_get_encoders");
  pmm_get_robot_v100_lidars= (pt_mm_get_robot_v100_lidars ) GetProcAddress(mmLibrary, "mm_robotv100_get_lidars");
  pmm_get_robot_v100_location= (pt_mm_get_robot_v100_location ) GetProcAddress(mmLibrary, "mm_robotv100_get_location");
  pmm_get_robot_v100_raw_imu= (pt_mm_get_robot_v100_raw_imu ) GetProcAddress(mmLibrary, "mm_robotv100_get_raw_imu");

  pmm_set_robot_v100_motors= (pt_mm_set_robot_v100_motors ) GetProcAddress(mmLibrary, "mm_robotv100_set_motors");

  pmm_device_is_modem= (pt_mm_device_is_modem ) GetProcAddress(mmLibrary, "mm_device_is_modem");
  pmm_device_is_beacon= (pt_mm_device_is_beacon ) GetProcAddress(mmLibrary, "mm_device_is_beacon");
  pmm_device_is_hedgehog= (pt_mm_device_is_hedgehog ) GetProcAddress(mmLibrary, "mm_device_is_hedgehog");
  pmm_device_is_robot= (pt_mm_device_is_robot ) GetProcAddress(mmLibrary, "mm_device_is_robot");
#else
// not WIN32
  if (dll_filepath == NULL) {
       mmLibrary = dlopen("libdashapi.so", RTLD_LAZY);
   }
   else {
       mmLibrary = dlopen(dll_filepath, RTLD_LAZY);
   }

  pmm_api_version= (pt_mm_api_version ) (intptr_t) dlsym(mmLibrary, "mm_api_version");
  pmm_get_last_error= (pt_mm_get_last_error ) (intptr_t) dlsym(mmLibrary, "mm_get_last_error");

  pmm_open_port= (pt_mm_open_port ) (intptr_t) dlsym(mmLibrary, "mm_open_port");
  pmm_open_port_by_name= (pt_mm_open_port_by_name ) (intptr_t) dlsym(mmLibrary, "mm_open_port_by_name");
  pmm_open_port_udp= (pt_mm_open_port_udp ) (intptr_t) dlsym(mmLibrary, "mm_open_port_udp");
  pmm_close_port= (pt_mm_close_port ) (intptr_t) dlsym(mmLibrary, "mm_close_port");

  pmm_get_version_and_id= (pt_mm_get_version_and_id ) (intptr_t) dlsym(mmLibrary, "mm_get_device_version_and_id");
  pmm_get_devices_list= (pt_mm_get_devices_list ) (intptr_t) dlsym(mmLibrary, "mm_get_devices_list");

  pmm_wake_device= (pt_mm_wake_device ) (intptr_t) dlsym(mmLibrary, "mm_wake_device");
  pmm_sleep_device= (pt_mm_sleep_device ) (intptr_t) dlsym(mmLibrary, "mm_send_to_sleep_device");

  pmm_get_beacon_tele= (pt_mm_get_beacon_tele ) (intptr_t) dlsym(mmLibrary, "mm_get_beacon_telemetry");

  pmm_get_last_locations= (pt_mm_get_last_locations ) (intptr_t) dlsym(mmLibrary, "mm_get_last_locations");
  pmm_get_last_locations2= (pt_mm_get_last_locations2 ) (intptr_t) dlsym(mmLibrary, "mm_get_last_locations2");

  pmm_get_last_distances= (pt_mm_get_last_distances ) (intptr_t) dlsym(mmLibrary, "mm_get_last_distances");

  pmm_get_update_rate_setting= (pt_mm_get_update_rate_setting ) (intptr_t) dlsym(mmLibrary, "mm_get_update_rate_setting");
  pmm_set_update_rate_setting= (pt_mm_set_update_rate_setting ) (intptr_t) dlsym(mmLibrary, "mm_set_update_rate_setting");

  pmm_add_submap= (pt_mm_add_submap ) (intptr_t) dlsym(mmLibrary, "mm_add_submap");
  pmm_delete_submap= (pt_mm_delete_submap ) (intptr_t) dlsym(mmLibrary, "mm_delete_submap");

  pmm_freeze_submap= (pt_mm_freeze_submap ) (intptr_t) dlsym(mmLibrary, "mm_freeze_submap");
  pmm_unfreeze_submap= (pt_mm_unfreeze_submap ) (intptr_t) dlsym(mmLibrary, "mm_unfreeze_submap");

  pmm_get_submap_settings= (pt_mm_get_submap_settings ) (intptr_t) dlsym(mmLibrary, "mm_get_submap_settings");
  pmm_set_submap_settings= (pt_mm_set_submap_settings ) (intptr_t) dlsym(mmLibrary, "mm_set_submap_settings");

  pmm_get_ultrasound_settings= (pt_mm_get_ultrasound_settings ) (intptr_t) dlsym(mmLibrary, "mm_get_ultrasound_settings");
  pmm_set_ultrasound_settings= (pt_mm_set_ultrasound_settings ) (intptr_t) dlsym(mmLibrary, "mm_set_ultrasound_settings");

  pmm_erase_map= (pt_mm_erase_map ) (intptr_t) dlsym(mmLibrary, "mm_erase_map");
  pmm_set_default_settings= (pt_mm_set_default_settings ) (intptr_t) dlsym(mmLibrary, "mm_set_default_settings");

  pmm_freeze_map= (pt_mm_freeze_map ) (intptr_t) dlsym(mmLibrary, "mm_freeze_map");
  pmm_unfreeze_map= (pt_mm_unfreeze_map ) (intptr_t) dlsym(mmLibrary, "mm_unfreeze_map");

  pmm_beacons_to_axes= (pt_mm_beacons_to_axes ) (intptr_t) dlsym(mmLibrary, "mm_beacons_to_axes");

  pmm_read_flash_dump= (pt_read_flash_dump ) (intptr_t) dlsym(mmLibrary, "mm_read_flash_dump");
  pmm_write_flash_dump= (pt_write_flash_dump ) (intptr_t) dlsym(mmLibrary, "mm_write_flash_dump");

  pmm_reset_device= (pt_mm_reset_device ) (intptr_t) dlsym(mmLibrary, "mm_reset_device");

  pmm_get_air_temperature= (pt_mm_get_air_temperature ) (intptr_t) dlsym(mmLibrary, "mm_get_air_temperature");
  pmm_set_air_temperature= (pt_mm_set_air_temperature ) (intptr_t) dlsym(mmLibrary, "mm_set_air_temperature");

  pmm_set_beacon_location= (pt_mm_set_beacon_location ) (intptr_t) dlsym(mmLibrary, "mm_set_beacon_location");
  pmm_set_beacons_distance= (pt_mm_set_beacons_distance ) (intptr_t) dlsym(mmLibrary, "mm_set_beacons_distance");

  pmm_get_hedge_height= (pt_mm_get_hedge_height ) (intptr_t) dlsym(mmLibrary, "mm_get_hedge_height");
  pmm_set_hedge_height= (pt_mm_set_hedge_height ) (intptr_t) dlsym(mmLibrary, "mm_set_hedge_height");
  pmm_get_beacon_height= (pt_mm_get_beacon_height ) (intptr_t) dlsym(mmLibrary, "mm_get_beacon_height");
  pmm_set_beacon_height= (pt_mm_set_beacon_height ) (intptr_t) dlsym(mmLibrary, "mm_set_beacon_height");

  pmm_get_realtime_player_settings= (pt_mm_get_realtime_player_settings ) (intptr_t) dlsym(mmLibrary, "mm_get_realtime_player_settings");
  pmm_set_realtime_player_settings= (pt_mm_set_realtime_player_settings ) (intptr_t) dlsym(mmLibrary, "mm_set_realtime_player_settings");

  pmm_get_georeferencing_settings= (pt_mm_get_georeferencing_settings ) (intptr_t) dlsym(mmLibrary, "mm_get_georeferencing_settings");
  pmm_set_georeferencing_settings= (pt_mm_set_georeferencing_settings ) (intptr_t) dlsym(mmLibrary, "mm_set_georeferencing_settings");

  pmm_get_update_positions_mode= (pt_mm_get_update_positions_mode ) (intptr_t) dlsym(mmLibrary, "mm_get_update_position_mode");
  pmm_set_update_positions_mode= (pt_mm_set_update_positions_mode ) (intptr_t) dlsym(mmLibrary, "mm_set_update_position_mode");
  pmm_set_update_positions_command= (pt_mm_set_update_positions_command ) (intptr_t) dlsym(mmLibrary, "mm_set_update_position_command");

  pmm_set_alarm_state_command= (pt_mm_set_alarm_state ) (intptr_t) dlsym(mmLibrary, "mm_set_alarm_state");
  pmm_pt_send_user_payload= (pt_mm_send_user_payload ) (intptr_t) dlsym(mmLibrary, "mm_send_user_payload_data");
  pmm_pt_get_user_payload= (pt_mm_get_user_payload ) (intptr_t) dlsym(mmLibrary, "mm_get_user_payload_data");

  pmm_set_motors_control= (pt_mm_set_motors_control ) (intptr_t) dlsym(mmLibrary, "mm_set_robot_motors_control");
  pmm_set_robot_program_item= (pt_mm_set_robot_program_item ) (intptr_t) dlsym(mmLibrary, "mm_set_robot_program_item");
  pmm_set_robot_command= (pt_mm_set_robot_command ) (intptr_t) dlsym(mmLibrary, "mm_set_robot_command");
  pmm_set_robot_position= (pt_mm_set_robot_position ) (intptr_t) dlsym(mmLibrary, "mm_set_robot_position");
  pmm_get_robot_telemetry= (pt_mm_get_robot_telemetry ) (intptr_t) dlsym(mmLibrary, "mm_get_robot_telemetry");

  pmm_get_robot_settings= (pt_mm_get_robot_settings ) (intptr_t) dlsym(mmLibrary, "mm_get_robot_settings");
  pmm_set_robot_settings=  (pt_mm_set_robot_settings ) (intptr_t) dlsym(mmLibrary, "mm_set_robot_settings");

  pmm_get_robot_v100_power= (pt_mm_get_robot_v100_power ) (intptr_t) dlsym(mmLibrary, "mm_robotv100_get_power");
  pmm_get_robot_v100_encoders= (pt_mm_get_robot_v100_encoders ) (intptr_t) dlsym(mmLibrary, "mm_robotv100_get_encoders");
  pmm_get_robot_v100_lidars= (pt_mm_get_robot_v100_lidars ) (intptr_t) dlsym(mmLibrary, "mm_robotv100_get_lidars");
  pmm_get_robot_v100_location= (pt_mm_get_robot_v100_location ) (intptr_t) dlsym(mmLibrary, "mm_robotv100_get_location");
  pmm_get_robot_v100_raw_imu= (pt_mm_get_robot_v100_raw_imu )  (intptr_t) dlsym(mmLibrary, "mm_robotv100_get_raw_imu");

  pmm_set_robot_v100_motors= (pt_mm_set_robot_v100_motors ) (intptr_t) dlsym(mmLibrary, "mm_robotv100_set_motors");

  pmm_device_is_modem= (pt_mm_device_is_modem ) (intptr_t) dlsym(mmLibrary, "mm_device_is_modem");
  pmm_device_is_beacon= (pt_mm_device_is_beacon ) (intptr_t) dlsym(mmLibrary, "mm_device_is_beacon");
  pmm_device_is_hedgehog= (pt_mm_device_is_hedgehog ) (intptr_t) dlsym(mmLibrary, "mm_device_is_hedgehog");
  pmm_device_is_robot= (pt_mm_device_is_robot ) (intptr_t) dlsym(mmLibrary, "mm_device_is_robot");
#endif
}


void marvelmindAPIFree() {
#ifdef WIN32
  FreeLibrary(mmLibrary);
#else
  dlclose(mmLibrary);
#endif
}
