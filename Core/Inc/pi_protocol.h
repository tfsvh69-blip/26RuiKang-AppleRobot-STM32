#ifndef __PI_PROTOCOL_H__
#define __PI_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include "fruit_pick.h"

#define PI_PROTOCOL_LINE_BUF_LEN      128u

/**
  * @brief 果树观察方向。
  */
typedef enum
{
  TREE_VIEW_LEFT = 0,
  TREE_VIEW_RIGHT = 1
} TreeViewId_t;

/**
  * @brief 树莓派协议返回值。
  */
typedef enum
{
  PI_OK = 0,
  PI_ERR_TIMEOUT = -1,
  PI_ERR_PARSE = -2,
  PI_ERR_NONE = -3,
  PI_ERR_REMOTE = -4,
  PI_ERR_OUT_OF_RANGE = -5,
  PI_ERR_PARAM = -6
} PiResult_t;

int Pi_Ping(uint16_t seq, uint32_t timeout_ms);

int Pi_RequestBestFruit(uint8_t tree_id,
                        TreeViewId_t view_id,
                        FruitTarget_t *out,
                        uint32_t timeout_ms);

int Pi_StartWatch(uint8_t tree_id,
                  TreeViewId_t view_id,
                  uint16_t *watch_seq,
                  uint32_t timeout_ms);

int Pi_PollWatchHit(uint16_t watch_seq,
                    FruitTarget_t *out,
                    uint32_t timeout_ms);

int Pi_StopWatch(uint16_t watch_seq);

int Pi_ParseLine(const char *line,
                 uint16_t expected_seq,
                 FruitTarget_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __PI_PROTOCOL_H__ */
