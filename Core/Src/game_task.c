#include "game_task.h"
#include "base_control.h"
#include "fruit_pick.h"
#include "cmsis_os.h"

#define GAME_PI_SCAN_TIMEOUT_MS       (10000u)
#define GAME_CREEP_STOP_SETTLE_MS     (500u)

int Game_CreepWatchAndPick(uint8_t tree_id,
                           TreeViewId_t view_id,
                           float total_distance_cm,
                           int16_t speed_rrp,
                           float yaw_abs,
                           float kp,
                           float kd)
{
  float traveled_cm = 0.0f;
  float step_traveled_cm;
  float remaining_cm;
  int move_status;
  int pi_status;
  int pick_status;
  FruitTarget_t fruit = {FRUIT_BIG, 0, 0, 0, 0, 0};

  if ((total_distance_cm <= 0.0f) || (speed_rrp == 0))
  {
    return -1;
  }

  while (traveled_cm < total_distance_cm)
  {
    remaining_cm = total_distance_cm - traveled_cm;
    step_traveled_cm = 0.0f;

    move_status = Base_ForwardDistanceCmHoldYawWatchPi(remaining_cm,
                                                       speed_rrp,
                                                       yaw_abs,
                                                       kp,
                                                       kd,
                                                       tree_id,
                                                       view_id,
                                                       &step_traveled_cm);
    traveled_cm += step_traveled_cm;

    if (move_status == 0)
    {
      return 0;
    }

    if (move_status != 1)
    {
      return move_status;
    }

    osDelay(GAME_CREEP_STOP_SETTLE_MS);
    pi_status = Pi_RequestBestFruit(tree_id,
                                    view_id,
                                    &fruit,
                                    GAME_PI_SCAN_TIMEOUT_MS);
    if (pi_status == PI_ERR_NONE)
    {
      continue;
    }
    if (pi_status != PI_OK)
    {
      return pi_status;
    }

    pick_status = FruitPick_PickOne(&fruit);
    if (pick_status != FRUIT_PICK_OK)
    {
      return pick_status;
    }
  }

  return 0;
}
