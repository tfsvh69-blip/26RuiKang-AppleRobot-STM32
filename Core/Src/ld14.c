#include "ld14.h"

#include <string.h>

#define LD14_HEADER           (0xAAu)
#define LD14_DEVICE_ADDRESS   (0x00u)
#define LD14_CHUNK_OFFSET     (0x00u)
#define LD14_CMD_GET_DISTANCE (0x02u)

typedef struct
{
  uint16_t distance;
  uint16_t noise;
  uint32_t peak;
  uint8_t confidence;
  uint32_t intg;
  int16_t reftof;
} LD14_Point_t;

/* 解析结果（平均值） */
volatile uint16_t g_ld14_distance_mm = 0;
volatile uint16_t g_ld14_noise = 0;
volatile uint32_t g_ld14_peak = 0;
volatile uint8_t g_ld14_confidence = 0;
volatile uint32_t g_ld14_intg = 0;
volatile int16_t g_ld14_reftof = 0;

/* 调试计数器：用于定位“有没有在收/有没有通过CRC” */
volatile uint32_t g_ld14_rx_bytes_cnt = 0;
volatile uint32_t g_ld14_frame_ok_cnt = 0;
volatile uint32_t g_ld14_frame_crc_err_cnt = 0;
volatile uint8_t g_ld14_last_crc_calc = 0;
volatile uint8_t g_ld14_last_crc_recv = 0;

static LD14_Point_t s_points[12];

static void ld14_process_frame_and_store(void)
{
  uint32_t sum_distance = 0;
  uint32_t sum_noise = 0;
  uint64_t sum_peak = 0;
  uint32_t sum_conf = 0;
  uint64_t sum_intg = 0;
  int32_t sum_reftof = 0;
  uint32_t count = 0;

  for (uint32_t i = 0; i < 12u; i++)
  {
    if (s_points[i].distance != 0u)
    {
      count++;
      sum_distance += s_points[i].distance;
      sum_noise += s_points[i].noise;
      sum_peak += s_points[i].peak;
      sum_conf += s_points[i].confidence;
      sum_intg += s_points[i].intg;
      sum_reftof += s_points[i].reftof;
    }
  }

  if (count == 0u)
    return;

  g_ld14_distance_mm = (uint16_t)(sum_distance / count);
  g_ld14_noise = (uint16_t)(sum_noise / count);
  g_ld14_peak = (uint32_t)(sum_peak / count);
  g_ld14_confidence = (uint8_t)(sum_conf / count);
  g_ld14_intg = (uint32_t)(sum_intg / count);
  g_ld14_reftof = (int16_t)(sum_reftof / (int32_t)count);
}

/* 将状态机变量放到文件作用域，保证 Reset 可控 */
static uint8_t s_hdr_cnt = 0;
static uint16_t s_state = 0;
static uint8_t s_crc = 0;
static uint8_t s_point_idx = 0;
static uint8_t s_field_idx = 0;
static uint8_t s_pack_ok = 0;
static uint8_t s_ts_idx = 0;

static void ld14_reset_state(void)
{
  s_hdr_cnt = 0u;
  s_state = 0u;
  s_crc = 0u;
  s_point_idx = 0u;
  s_field_idx = 0u;
  s_pack_ok = 0u;
  s_ts_idx = 0u;
  memset(s_points, 0, sizeof(s_points));
}

void LD14_Reset(void)
{
  ld14_reset_state();
}

void LD14_FeedByte(uint8_t b)
{
  g_ld14_rx_bytes_cnt++;

  /* 1) 帧头：连续4个0xAA */
  if (s_state == 0u)
  {
    if (b == LD14_HEADER)
    {
      s_hdr_cnt++;
      if (s_hdr_cnt >= 4u)
      {
        s_state = 1u;
        s_hdr_cnt = 0u;
        s_crc = 0u;
        s_pack_ok = 0u;
      }
    }
    else
    {
      s_hdr_cnt = 0u;
    }
    return;
  }

  /* 2) 固定头字段：addr/cmd/off1/off2/lenL/lenH */
  switch (s_state)
  {
    case 1: /* addr */
      if (b != LD14_DEVICE_ADDRESS)
      {
        ld14_reset_state();
        return;
      }
      s_crc = (uint8_t)(s_crc + b);
      s_state = 2u;
      return;

    case 2: /* cmd */
      if (b != LD14_CMD_GET_DISTANCE)
      {
        ld14_reset_state();
        return;
      }
      s_crc = (uint8_t)(s_crc + b);
      s_state = 3u;
      return;

    case 3: /* offset low */
      if (b != LD14_CHUNK_OFFSET)
      {
        ld14_reset_state();
        return;
      }
      s_crc = (uint8_t)(s_crc + b);
      s_state = 4u;
      return;

    case 4: /* offset high */
      if (b != LD14_CHUNK_OFFSET)
      {
        ld14_reset_state();
        return;
      }
      s_crc = (uint8_t)(s_crc + b);
      s_state = 5u;
      return;

    case 5: /* lenL */
      s_crc = (uint8_t)(s_crc + b);
      s_state = 6u;
      return;

    case 6: /* lenH */
      s_crc = (uint8_t)(s_crc + b);
      /* 协议此处长度固定，后续按固定长度读 */
      s_state = 7u;
      s_point_idx = 0u;
      s_field_idx = 0u;
      s_pack_ok = 1u;
      return;

    default:
      break;
  }

  if (!s_pack_ok)
  {
    ld14_reset_state();
    return;
  }

  /* 3) payload：12个点，每点15字节 */
  if (s_state == 7u)
  {
    /* 每个点字段序号 0..14 */
    LD14_Point_t *p = &s_points[s_point_idx];

    switch (s_field_idx)
    {
      case 0: p->distance = (uint16_t)b; break;
      case 1: p->distance = (uint16_t)(p->distance | ((uint16_t)b << 8)); break;
      case 2: p->noise = (uint16_t)b; break;
      case 3: p->noise = (uint16_t)(p->noise | ((uint16_t)b << 8)); break;
      case 4: p->peak = (uint32_t)b; break;
      case 5: p->peak |= ((uint32_t)b << 8); break;
      case 6: p->peak |= ((uint32_t)b << 16); break;
      case 7: p->peak |= ((uint32_t)b << 24); break;
      case 8: p->confidence = b; break;
      case 9: p->intg = (uint32_t)b; break;
      case 10: p->intg |= ((uint32_t)b << 8); break;
      case 11: p->intg |= ((uint32_t)b << 16); break;
      case 12: p->intg |= ((uint32_t)b << 24); break;
      case 13: p->reftof = (int16_t)b; break;
      case 14: p->reftof = (int16_t)(p->reftof | ((int16_t)b << 8)); break;
      default: break;
    }

    s_crc = (uint8_t)(s_crc + b);

    s_field_idx++;
    if (s_field_idx >= 15u)
    {
      s_field_idx = 0u;
      s_point_idx++;
      if (s_point_idx >= 12u)
      {
        s_state = 8u;
        s_ts_idx = 0u;
      }
    }
    return;
  }

  /* 4) timestamp：4字节 */
  if (s_state == 8u)
  {
    s_ts_idx++;
    s_crc = (uint8_t)(s_crc + b);

    if (s_ts_idx >= 4u)
    {
      s_state = 9u;
    }
    return;
  }

  /* 5) CRC：1字节 */
  if (s_state == 9u)
  {
    g_ld14_last_crc_calc = s_crc;
    g_ld14_last_crc_recv = b;

    if (b == s_crc)
    {
      ld14_process_frame_and_store();
      g_ld14_frame_ok_cnt++;
    }
    else
    {
      g_ld14_frame_crc_err_cnt++;
    }

    /* 下一帧 */
    ld14_reset_state();
    return;
  }

  /* 兜底 */
  ld14_reset_state();
}
