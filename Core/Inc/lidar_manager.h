#ifndef __LIDAR_MANAGER_H__
#define __LIDAR_MANAGER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* LD14 单点数据 */
typedef struct {
    int16_t distance;
    uint16_t noise;
    uint32_t peak;
    uint8_t confidence;
    uint32_t intg;
    int16_t reftof;
} LidarPoint_t;

/* 帧数据 */
typedef struct {
    LidarPoint_t points[12];
    uint32_t timestamp;
    uint8_t is_updated;
    uint8_t status;
} LidarSensor_t;

/* 数组 */
extern LidarSensor_t g_LidarArray[4];

/* 解析单帧 */
void Lidar_ParseFrame(uint8_t sensor_id, uint8_t *raw_buf, uint16_t buf_len);

/* 解析数据流 */
void Lidar_ProcessStream(uint8_t sensor_id, const uint8_t *stream, uint16_t len);

/* 获取最新帧 */
uint8_t Lidar_GetData(uint8_t sensor_id, LidarSensor_t *out_data);

#ifdef __cplusplus
}
#endif

#endif
