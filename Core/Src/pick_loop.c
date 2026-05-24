#include "pick_loop.h"
#include "cmsis_os.h"

#define PICK_LOOP_DEFAULT_RETRY_DELAY_MS     200u

static void PickLoop_DelayMs(uint32_t delay_ms)
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

int PickLoop_RunCurrentView(const PickLoopConfig_t *config, uint8_t *picked_count)
{
  FruitTarget_t fruit;
  uint8_t picked;
  uint8_t max_count;
  uint32_t retry_delay;
  int ret;

  if ((config == NULL) || (config->request_fruit == NULL))
  {
    return PICK_LOOP_ERR_PARAM;
  }

  picked = 0u;
  max_count = config->max_pick_count;
  retry_delay = config->retry_delay_ms;

  if (max_count == 0u)
  {
    return PICK_LOOP_ERR_PARAM;
  }

  if (retry_delay == 0u)
  {
    retry_delay = PICK_LOOP_DEFAULT_RETRY_DELAY_MS;
  }

  while (picked < max_count)
  {
    ret = config->request_fruit(&fruit, config->context);
    if (ret != 0)
    {
      if (picked_count != NULL)
      {
        *picked_count = picked;
      }

      return (picked == 0u) ? PICK_LOOP_ERR_NO_FRUIT : PICK_LOOP_OK;
    }

    ret = FruitPick_PickOne(&fruit);
    if (ret != FRUIT_PICK_OK)
    {
      if (picked_count != NULL)
      {
        *picked_count = picked;
      }

      return PICK_LOOP_ERR_PICK;
    }

    picked++;
    PickLoop_DelayMs(retry_delay);
  }

  if (picked_count != NULL)
  {
    *picked_count = picked;
  }

  return PICK_LOOP_OK;
}
