#ifndef CNR_MCSDK_POSITION_CONTROL_TEST_H
#define CNR_MCSDK_POSITION_CONTROL_TEST_H

#define CNR_POSITION_CONTROL_POC_ENABLE    0U

#define CNR_FOLLOW_LOG_INTERVAL_MS    1U
#define CNR_FOLLOW_SETTLE_TIME_MS    5000U

/* Position Control */
#define POS_CTRL_100RAD_TARGET     100.0f
#define POS_CTRL_1000RAD_TARGET    1000.0f
#define FOLLOW_TIMEOUT_MS    4000U
#define FOLLOW_MODE_DURATION_S    0.0f

/* Trajectory Control */
#define TRAJ_CTRL_100RAD_TARGET    100.0f
#define TRAJ_CTRL_100RAD_OUT_TIME  2.0f
#define TRAJ_CTRL_100RAD_BACK_TIME 2.0f
#define TRAJ_CTRL_1000RAD_TARGET    1000.0f
#define TRAJ_CTRL_1000RAD_OUT_TIME  2.0f
#define TRAJ_CTRL_1000RAD_BACK_TIME 4.0f




void cnr_mcsdk_position_control_poc_test(void);
void cnr_follow_capture_log(void);

#endif /* CNR_MCSDK_POSITION_CONTROL_TEST_H */