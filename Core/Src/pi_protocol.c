#include "pi_protocol.h"
#include "Usart_to_Pi.h"
#include "arm_motion.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI_PROTOCOL_DIAMETER_MIN_MM    40
#define PI_PROTOCOL_DIAMETER_MAX_MM    100
#define PI_PROTOCOL_SCORE_MIN          0
#define PI_PROTOCOL_SCORE_MAX          100
#define PI_PROTOCOL_SEQ_START          2u

static uint16_t s_pi_protocol_seq = PI_PROTOCOL_SEQ_START;
volatile PiNoneReason_t g_pi_debug_last_none_reason = PI_NONE_REASON_UNKNOWN;
volatile uint8_t g_pi_debug_last_hit_stable = 0u;
volatile PiHitZone_t g_pi_debug_last_hit_zone = PI_HIT_ZONE_UNKNOWN;

/*
 * 协议调试变量。
 * Keil Debug 的 Watch 窗口可以直接查看：
 *   g_pi_debug_last_rx_line       最近一次拿去解析的文本行
 *   g_pi_debug_last_tx_line       最近一次 STM32 发给树莓派的文本行
 *   g_pi_debug_expected_seq       本次等待的 seq
 *   g_pi_debug_parse_step         解析失败/成功位置
 *   g_pi_debug_parse_ret          最近一次 Pi_ParseLine() 返回值
 *   g_pi_debug_rx_len/c0..c15     最近一次接收行长度和前 16 个字符 ASCII 码
 */
static void PiProtocol_DelayMs(uint32_t delay_ms)
{
  if (delay_ms == 0u)
  {
    return;
  }

  if (osKernelGetState() == osKernelRunning)
  {
    osDelay(delay_ms);
  }
  else
  {
    HAL_Delay(delay_ms);
  }
}

static uint32_t PiProtocol_GetTickMs(void)
{
  return HAL_GetTick();
}

static bool PiProtocol_IsTimeout(uint32_t start_ms, uint32_t timeout_ms)
{
  return ((uint32_t)(PiProtocol_GetTickMs() - start_ms) >= timeout_ms);
}

static uint16_t PiProtocol_NextSeq(void)
{
  uint16_t seq;

  seq = s_pi_protocol_seq++;
  if (s_pi_protocol_seq == 0u)
  {
    s_pi_protocol_seq = PI_PROTOCOL_SEQ_START;
  }

  return seq;
}

static int PiProtocol_ViewToChar(TreeViewId_t view_id, char *view_char)
{
  if (view_char == NULL)
  {
    return PI_ERR_PARAM;
  }

  if (view_id == TREE_VIEW_LEFT)
  {
    *view_char = 'L';
    return PI_OK;
  }

  if (view_id == TREE_VIEW_RIGHT)
  {
    *view_char = 'R';
    return PI_OK;
  }

  return PI_ERR_PARAM;
}

static int PiProtocol_SendText(const char *text)
{
  uint16_t len;

  if (text == NULL)
  {
    return PI_ERR_PARAM;
  }

  len = (uint16_t)strlen(text);
  if ((len == 0u) || (len >= PI_UART_TX_BUF_LEN))
  {
    return PI_ERR_PARAM;
  }

  if (PiUart2_Send((const uint8_t *)text, len) == false)
  {
    return PI_ERR_TIMEOUT;
  }

  return PI_OK;
}

static int PiProtocol_WaitLine(char *line, uint16_t line_len, uint32_t timeout_ms)
{
  static uint8_t s_pending_frame[PI_UART_RX_BUF_LEN];
  static uint16_t s_pending_frame_len = 0u;
  static uint16_t s_pending_frame_pos = 0u;
  static char s_line_acc[PI_PROTOCOL_LINE_BUF_LEN];
  static uint16_t s_line_acc_len = 0u;
  uint8_t ch;
  uint16_t copy_len;
  uint32_t start_ms;

  if ((line == NULL) || (line_len == 0u))
  {
    return PI_ERR_PARAM;
  }

  start_ms = PiProtocol_GetTickMs();
  while (PiProtocol_IsTimeout(start_ms, timeout_ms) == false)
  {
    if (s_pending_frame_pos >= s_pending_frame_len)
    {
      if (PiUart2_GetFrame(s_pending_frame,
                           (uint16_t)sizeof(s_pending_frame),
                           &s_pending_frame_len) == true)
      {
        s_pending_frame_pos = 0u;
      }
    }

    while (s_pending_frame_pos < s_pending_frame_len)
    {
      ch = s_pending_frame[s_pending_frame_pos];
      s_pending_frame_pos++;

      if ((ch == (uint8_t)'\n') || (ch == (uint8_t)'\r'))
      {
        if (s_line_acc_len == 0u)
        {
          continue;
        }

        while ((s_line_acc_len > 0u) &&
               ((s_line_acc[s_line_acc_len - 1u] == ' ') ||
                (s_line_acc[s_line_acc_len - 1u] == '\t')))
        {
          s_line_acc_len--;
        }
        s_line_acc[s_line_acc_len] = '\0';

        copy_len = s_line_acc_len;
        if (copy_len > (uint16_t)(line_len - 1u))
        {
          copy_len = (uint16_t)(line_len - 1u);
        }
        (void)memcpy(line, s_line_acc, copy_len);
        line[copy_len] = '\0';
        s_line_acc_len = 0u;

        return PI_OK;
      }

      if (s_line_acc_len >= (uint16_t)(sizeof(s_line_acc) - 1u))
      {
        s_line_acc_len = 0u;
        return PI_ERR_PARSE;
      }

      s_line_acc[s_line_acc_len] = (char)ch;
      s_line_acc_len++;
    }

    /* 等待 '\n' 后再解析，避免 UART idle 半包被当成完整 HIT。 */
    PiProtocol_DelayMs(5u);
  }

  return PI_ERR_TIMEOUT;
}

static int PiProtocol_ParseInt32Token(char **cursor, int32_t *out)
{
  char *start;
  char *end;
  long value;

  if ((cursor == NULL) || (*cursor == NULL) || (out == NULL))
  {
    return PI_ERR_PARAM;
  }

  start = *cursor;
  value = strtol(start, &end, 10);
  if (end == start)
  {
    return PI_ERR_PARSE;
  }

  if ((*end != ',') && (*end != '\0'))
  {
    return PI_ERR_PARSE;
  }

  *out = (int32_t)value;
  *cursor = (*end == ',') ? (end + 1) : end;
  return PI_OK;
}

static int PiProtocol_ParseSeq(char **cursor, uint16_t expected_seq)
{
  int32_t seq_value;
  int ret;

  ret = PiProtocol_ParseInt32Token(cursor, &seq_value);
  if (ret != PI_OK)
  {
    return ret;
  }

  if ((seq_value < 0) || (seq_value > 65535) || ((uint16_t)seq_value != expected_seq))
  {
    return PI_ERR_PARSE;
  }

  return PI_OK;
}

static PiNoneReason_t PiProtocol_ParseNoneReason(const char *reason)
{
  if (reason == NULL)
  {
    return PI_NONE_REASON_UNKNOWN;
  }

  if (strcmp(reason, "NO_TARGET") == 0)
  {
    return PI_NONE_REASON_NO_TARGET;
  }
  if (strcmp(reason, "OUT_OF_RANGE") == 0)
  {
    return PI_NONE_REASON_OUT_OF_RANGE;
  }
  if (strcmp(reason, "UNKNOWN_SIZE") == 0)
  {
    return PI_NONE_REASON_UNKNOWN_SIZE;
  }
  if (strcmp(reason, "LOW_SCORE") == 0)
  {
    return PI_NONE_REASON_LOW_SCORE;
  }
  if (strcmp(reason, "OCCLUDED") == 0)
  {
    return PI_NONE_REASON_OCCLUDED;
  }

  return PI_NONE_REASON_UNKNOWN;
}

static int PiProtocol_ParseNone(char *cursor, uint16_t expected_seq)
{
  PiNoneReason_t reason;
  int ret;

  g_pi_debug_last_none_reason = PI_NONE_REASON_UNKNOWN;

  ret = PiProtocol_ParseSeq(&cursor, expected_seq);
  if (ret != PI_OK)
  {
    return ret;
  }

  if (*cursor == '\0')
  {
    g_pi_debug_last_none_reason = PI_NONE_REASON_NO_TARGET;
    return PI_ERR_NONE;
  }

  if (strchr(cursor, ',') != NULL)
  {
    return PI_ERR_PARSE;
  }

  reason = PiProtocol_ParseNoneReason(cursor);
  if (reason == PI_NONE_REASON_UNKNOWN)
  {
    return PI_ERR_PARSE;
  }

  g_pi_debug_last_none_reason = reason;
  return PI_ERR_NONE;
}

static int PiProtocol_ParseType(char **cursor, FruitType_t *type)
{
  char *start;
  char *end;

  if ((cursor == NULL) || (*cursor == NULL) || (type == NULL))
  {
    return PI_ERR_PARAM;
  }

  start = *cursor;
  end = strchr(start, ',');
  if (end == NULL)
  {
    return PI_ERR_PARSE;
  }

  if (((end - start) == 3) && (strncmp(start, "BIG", 3u) == 0))
  {
    *type = FRUIT_TYPE_BIG;
  }
  else if (((end - start) == 5) && (strncmp(start, "SMALL", 5u) == 0))
  {
    *type = FRUIT_TYPE_SMALL;
  }
  else
  {
    return PI_ERR_PARSE;
  }

  *cursor = end + 1;
  return PI_OK;
}

static PiHitZone_t PiProtocol_ParseHitZone(const char *zone_text)
{
  if (zone_text == NULL)
  {
    return PI_HIT_ZONE_UNKNOWN;
  }

  if (strcmp(zone_text, "EARLY") == 0)
  {
    return PI_HIT_ZONE_EARLY;
  }
  if (strcmp(zone_text, "GOOD") == 0)
  {
    return PI_HIT_ZONE_GOOD;
  }
  if (strcmp(zone_text, "LATE") == 0)
  {
    return PI_HIT_ZONE_LATE;
  }

  return PI_HIT_ZONE_UNKNOWN;
}

static int PiProtocol_CheckFruitRange(const FruitTarget_t *fruit)
{
  ArmPoint_t point;
  ArmMotionStatus_t arm_status;

  if (fruit == NULL)
  {
    return PI_ERR_PARAM;
  }

  if ((fruit->x_mm < 0) || (fruit->y_mm < 0) || (fruit->z_mm < 0))
  {
    return PI_ERR_OUT_OF_RANGE;
  }

  if (fruit->y_mm < (int32_t)ARM_MOTION_Y_PRE_EXTEND_MM)
  {
    return PI_ERR_OUT_OF_RANGE;
  }

  point.x_mm = (float)fruit->x_mm;
  point.y_mm = (float)fruit->y_mm;
  point.z_mm = (float)fruit->z_mm;

  arm_status = Arm_IsPointReachable(&point);
  if (arm_status != ARM_MOTION_OK)
  {
    return PI_ERR_OUT_OF_RANGE;
  }

  if ((fruit->diameter_mm < PI_PROTOCOL_DIAMETER_MIN_MM) ||
      (fruit->diameter_mm > PI_PROTOCOL_DIAMETER_MAX_MM))
  {
    return PI_ERR_PARSE;
  }

  if ((fruit->score < PI_PROTOCOL_SCORE_MIN) ||
      (fruit->score > PI_PROTOCOL_SCORE_MAX))
  {
    return PI_ERR_PARSE;
  }

  return PI_OK;
}

static int PiProtocol_ParseFruitLike(char *cursor, uint16_t expected_seq, FruitTarget_t *out)
{
  int32_t value;
  int ret;
  FruitTarget_t fruit;

  if (out == NULL)
  {
    return PI_ERR_PARSE;
  }

  ret = PiProtocol_ParseSeq(&cursor, expected_seq);
  if (ret != PI_OK)
  {
    return ret;
  }

  ret = PiProtocol_ParseType(&cursor, &fruit.type);
  if (ret != PI_OK)
  {
    return ret;
  }

  ret = PiProtocol_ParseInt32Token(&cursor, &fruit.x_mm);
  if (ret != PI_OK)
  {
    return ret;
  }

  ret = PiProtocol_ParseInt32Token(&cursor, &fruit.y_mm);
  if (ret != PI_OK)
  {
    return ret;
  }

  ret = PiProtocol_ParseInt32Token(&cursor, &fruit.z_mm);
  if (ret != PI_OK)
  {
    return ret;
  }

  ret = PiProtocol_ParseInt32Token(&cursor, &fruit.diameter_mm);
  if (ret != PI_OK)
  {
    return ret;
  }

  ret = PiProtocol_ParseInt32Token(&cursor, &value);
  if ((ret != PI_OK) || (*cursor != '\0'))
  {
    return PI_ERR_PARSE;
  }
  fruit.score = value;

  ret = PiProtocol_CheckFruitRange(&fruit);
  if (ret != PI_OK)
  {
    return ret;
  }

  *out = fruit;
  return PI_OK;
}

static int PiProtocol_ParseHit(char *cursor, uint16_t expected_seq, FruitTarget_t *out)
{
  int32_t score_value;
  int32_t stable_value;
  int ret;
  char *zone_text;
  FruitTarget_t hit;

  g_pi_debug_last_hit_stable = 0u;
  g_pi_debug_last_hit_zone = PI_HIT_ZONE_UNKNOWN;

  if (out == NULL)
  {
    return PI_ERR_PARSE;
  }

  ret = PiProtocol_ParseSeq(&cursor, expected_seq);
  if (ret != PI_OK)
  {
    return ret;
  }

  ret = PiProtocol_ParseType(&cursor, &hit.type);
  if (ret != PI_OK)
  {
    return ret;
  }

  ret = PiProtocol_ParseInt32Token(&cursor, &hit.x_mm);
  if (ret != PI_OK)
  {
    return ret;
  }

  ret = PiProtocol_ParseInt32Token(&cursor, &hit.y_mm);
  if (ret != PI_OK)
  {
    return ret;
  }

  ret = PiProtocol_ParseInt32Token(&cursor, &hit.z_mm);
  if (ret != PI_OK)
  {
    return ret;
  }

  ret = PiProtocol_ParseInt32Token(&cursor, &hit.diameter_mm);
  if (ret != PI_OK)
  {
    return ret;
  }

  ret = PiProtocol_ParseInt32Token(&cursor, &score_value);
  if (ret != PI_OK)
  {
    return ret;
  }
  hit.score = score_value;

  ret = PiProtocol_CheckFruitRange(&hit);
  if (ret != PI_OK)
  {
    return ret;
  }

  ret = PiProtocol_ParseInt32Token(&cursor, &stable_value);
  if (ret != PI_OK)
  {
    return ret;
  }
  if ((stable_value < 0) || (stable_value > 255))
  {
    return PI_ERR_PARSE;
  }

  zone_text = cursor;
  if ((zone_text == NULL) || (*zone_text == '\0') || (strchr(zone_text, ',') != NULL))
  {
    return PI_ERR_PARSE;
  }

  g_pi_debug_last_hit_stable = (uint8_t)stable_value;
  g_pi_debug_last_hit_zone = PiProtocol_ParseHitZone(zone_text);
  if (g_pi_debug_last_hit_zone == PI_HIT_ZONE_UNKNOWN)
  {
    return PI_ERR_PARSE;
  }

  *out = hit;
  return PI_OK;
}

int Pi_ParseLine(const char *line, uint16_t expected_seq, FruitTarget_t *out)
{
  char buf[PI_PROTOCOL_LINE_BUF_LEN];
  char *cursor;
  int ret;

  if (line == NULL)
  {
    return PI_ERR_PARAM;
  }

  if (strlen(line) >= sizeof(buf))
  {
    return PI_ERR_PARSE;
  }

  (void)strcpy(buf, line);

  if (strncmp(buf, "PONG,", 5u) == 0)
  {
    cursor = &buf[5];
    ret = PiProtocol_ParseSeq(&cursor, expected_seq);
    if ((ret == PI_OK) && (*cursor == '\0'))
    {
      return PI_OK;
    }

    return PI_ERR_PARSE;
  }

  if (strncmp(buf, "WATCHING,", 9u) == 0)
  {
    cursor = &buf[9];
    ret = PiProtocol_ParseSeq(&cursor, expected_seq);
    if ((ret == PI_OK) && (*cursor == '\0'))
    {
      return PI_OK;
    }

    return PI_ERR_PARSE;
  }

  if (strncmp(buf, "NONE,", 5u) == 0)
  {
    return PiProtocol_ParseNone(&buf[5], expected_seq);
  }

  if (strncmp(buf, "ERR,", 4u) == 0)
  {
    cursor = &buf[4];
    ret = PiProtocol_ParseSeq(&cursor, expected_seq);
    if (ret == PI_OK)
    {
      return PI_ERR_REMOTE;
    }

    return PI_ERR_PARSE;
  }

  if (strncmp(buf, "FRUIT,", 6u) == 0)
  {
    return PiProtocol_ParseFruitLike(&buf[6], expected_seq, out);
  }

  if (strncmp(buf, "HIT,", 4u) == 0)
  {
    return PiProtocol_ParseHit(&buf[4], expected_seq, out);
  }

  return PI_ERR_PARSE;
}

static bool PiProtocol_TryReadLineSeq(const char *line,
                                      const char *prefix,
                                      uint16_t *seq)
{
  const char *cursor;
  char *end;
  long value;
  size_t prefix_len;

  if ((line == NULL) || (prefix == NULL) || (seq == NULL))
  {
    return false;
  }

  prefix_len = strlen(prefix);
  if (strncmp(line, prefix, prefix_len) != 0)
  {
    return false;
  }

  cursor = line + prefix_len;
  value = strtol(cursor, &end, 10);
  if ((end == cursor) || (value < 0) || (value > 65535))
  {
    return false;
  }

  if ((*end != ',') && (*end != '\0'))
  {
    return false;
  }

  *seq = (uint16_t)value;
  return true;
}

static bool PiProtocol_ShouldIgnoreParseError(const char *line, uint16_t expected_seq)
{
  /*
   * 丢弃旧包/无关包：
   *   例如 SCAN,2 等待 FRUIT,2 时，串口里残留了 PONG,1。
   *   这种情况不能直接返回 PI_ERR_PARSE，应继续等当前 seq 的回复。
   *
   * 真正格式错误仍然返回 PI_ERR_PARSE：
   *   例如 FRUIT 当前 seq 对上了，但 type/x/y/z/diameter/score 格式错误。
   */
  uint16_t seq;

  if (line == NULL)
  {
    return false;
  }

  if (PiProtocol_TryReadLineSeq(line, "PONG,", &seq) == true)
  {
    return (seq != expected_seq);
  }

  if (PiProtocol_TryReadLineSeq(line, "WATCHING,", &seq) == true)
  {
    return (seq != expected_seq);
  }

  if (PiProtocol_TryReadLineSeq(line, "NONE,", &seq) == true)
  {
    return (seq != expected_seq);
  }

  if (PiProtocol_TryReadLineSeq(line, "ERR,", &seq) == true)
  {
    return (seq != expected_seq);
  }

  if (PiProtocol_TryReadLineSeq(line, "FRUIT,", &seq) == true)
  {
    return (seq != expected_seq);
  }

  if (PiProtocol_TryReadLineSeq(line, "HIT,", &seq) == true)
  {
    return (seq != expected_seq);
  }

  if ((strncmp(line, "PONG,", 5u) != 0) &&
      (strncmp(line, "WATCHING,", 9u) != 0) &&
      (strncmp(line, "NONE,", 5u) != 0) &&
      (strncmp(line, "ERR,", 4u) != 0) &&
      (strncmp(line, "FRUIT,", 6u) != 0) &&
      (strncmp(line, "HIT,", 4u) != 0))
  {
    return true;
  }

  return false;
}

static int PiProtocol_WaitParsedLine(uint16_t expected_seq,
                                     FruitTarget_t *out,
                                     uint32_t timeout_ms)
{
  char rx_line[PI_PROTOCOL_LINE_BUF_LEN];
  uint32_t start_ms;
  uint32_t elapsed_ms;
  uint32_t remain_ms;
  int ret;

  start_ms = PiProtocol_GetTickMs();
  while (PiProtocol_IsTimeout(start_ms, timeout_ms) == false)
  {
    elapsed_ms = (uint32_t)(PiProtocol_GetTickMs() - start_ms);
    remain_ms = timeout_ms - elapsed_ms;
    if (remain_ms == 0u)
    {
      break;
    }

    ret = PiProtocol_WaitLine(rx_line, (uint16_t)sizeof(rx_line), remain_ms);
    if (ret != PI_OK)
    {
      return ret;
    }

    ret = Pi_ParseLine(rx_line, expected_seq, out);
    if (ret == PI_OK)
    {
      return PI_OK;
    }

    if (ret != PI_ERR_PARSE)
    {
      return ret;
    }

    if (PiProtocol_ShouldIgnoreParseError(rx_line, expected_seq) == false)
    {
      return ret;
    }

    /*
     * 这里说明收到了旧 seq 或无关行，继续等待当前请求的回复。
     * 每次循环都会重新计算剩余 timeout。
     */
  }

  return PI_ERR_TIMEOUT;
}

int Pi_Ping(uint16_t seq, uint32_t timeout_ms)
{
  char tx_line[32];
  int ret;

  if (timeout_ms == 0u)
  {
    return PI_ERR_PARAM;
  }

  (void)snprintf(tx_line, sizeof(tx_line), "PING,%u\n", (unsigned int)seq);
  ret = PiProtocol_SendText(tx_line);
  if (ret != PI_OK)
  {
    return ret;
  }

  return PiProtocol_WaitParsedLine(seq, NULL, timeout_ms);
}

int Pi_RequestBestFruit(uint8_t tree_id,
                        TreeViewId_t view_id,
                        FruitTarget_t *out,
                        uint32_t timeout_ms)
{
  uint16_t scan_seq;

  return Pi_RequestBestFruitWithSeq(tree_id,
                                    view_id,
                                    out,
                                    timeout_ms,
                                    &scan_seq);
}

int Pi_RequestBestFruitWithSeq(uint8_t tree_id,
                               TreeViewId_t view_id,
                               FruitTarget_t *out,
                               uint32_t timeout_ms,
                               uint16_t *scan_seq_out)
{
  char tx_line[48];
  char view_char;
  uint16_t seq;
  int ret;

  if ((out == NULL) || (scan_seq_out == NULL) || (timeout_ms == 0u))
  {
    return PI_ERR_PARAM;
  }

  *scan_seq_out = 0u;
  g_pi_debug_last_none_reason = PI_NONE_REASON_UNKNOWN;

  ret = PiProtocol_ViewToChar(view_id, &view_char);
  if (ret != PI_OK)
  {
    return ret;
  }

  seq = PiProtocol_NextSeq();
  *scan_seq_out = seq;

  (void)snprintf(tx_line,
                 sizeof(tx_line),
                 "SCAN,%u,%u,%c\n",
                 (unsigned int)seq,
                 (unsigned int)tree_id,
                 view_char);

  ret = PiProtocol_SendText(tx_line);
  if (ret != PI_OK)
  {
    return ret;
  }

  return PiProtocol_WaitParsedLine(seq, out, timeout_ms);
}

int Pi_CancelScan(uint16_t scan_seq)
{
  char tx_line[32];

  if (scan_seq == 0u)
  {
    return PI_ERR_PARAM;
  }

  (void)snprintf(tx_line,
                 sizeof(tx_line),
                 "SCAN_CANCEL,%u\n",
                 (unsigned int)scan_seq);

  return PiProtocol_SendText(tx_line);
}

PiNoneReason_t Pi_GetLastNoneReason(void)
{
  return g_pi_debug_last_none_reason;
}

int Pi_StartWatch(uint8_t tree_id,
                  TreeViewId_t view_id,
                  uint16_t *watch_seq,
                  uint32_t timeout_ms)
{
  char tx_line[48];
  char view_char;
  uint16_t seq;
  int ret;

  if ((watch_seq == NULL) || (timeout_ms == 0u))
  {
    return PI_ERR_PARAM;
  }

  ret = PiProtocol_ViewToChar(view_id, &view_char);
  if (ret != PI_OK)
  {
    return ret;
  }

  seq = PiProtocol_NextSeq();
  (void)snprintf(tx_line,
                 sizeof(tx_line),
                 "WATCH,%u,%u,%c\n",
                 (unsigned int)seq,
                 (unsigned int)tree_id,
                 view_char);

  ret = PiProtocol_SendText(tx_line);
  if (ret != PI_OK)
  {
    return ret;
  }

  ret = PiProtocol_WaitParsedLine(seq, NULL, timeout_ms);
  if (ret != PI_OK)
  {
    return ret;
  }

  *watch_seq = seq;
  return PI_OK;
}

int Pi_PollWatchHit(uint16_t watch_seq,
                    FruitTarget_t *out,
                    uint32_t timeout_ms)
{
  char rx_line[PI_PROTOCOL_LINE_BUF_LEN];
  uint32_t wait_ms = (timeout_ms == 0u) ? 1u : timeout_ms;
  int ret;

  if (out == NULL)
  {
    return PI_ERR_PARAM;
  }

  ret = PiProtocol_WaitLine(rx_line, (uint16_t)sizeof(rx_line), wait_ms);
  if (ret != PI_OK)
  {
    return ret;
  }

  if (strncmp(rx_line, "HIT,", 4u) != 0)
  {
    if (strncmp(rx_line, "ERR,", 4u) == 0)
    {
      ret = Pi_ParseLine(rx_line, watch_seq, out);
      if (ret == PI_ERR_REMOTE)
      {
        return ret;
      }
    }

    return PI_ERR_TIMEOUT;
  }

  ret = Pi_ParseLine(rx_line, watch_seq, out);
  if (ret == PI_ERR_PARSE)
  {
    if (PiProtocol_ShouldIgnoreParseError(rx_line, watch_seq) == true)
    {
      return PI_ERR_TIMEOUT;
    }
  }

  if (ret == PI_OK)
  {
    if ((g_pi_debug_last_hit_stable < 2u) ||
        (g_pi_debug_last_hit_zone != PI_HIT_ZONE_GOOD))
    {
      return PI_ERR_TIMEOUT;
    }
  }

  return ret;
}

int Pi_StopWatch(uint16_t watch_seq)
{
  char tx_line[32];

  (void)snprintf(tx_line,
                 sizeof(tx_line),
                 "WATCH_STOP,%u\n",
                 (unsigned int)watch_seq);

  return PiProtocol_SendText(tx_line);
}
