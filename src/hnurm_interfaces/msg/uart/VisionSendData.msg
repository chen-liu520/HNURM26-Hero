std_msgs/Header header

TargetState target_state
TargetType target_type
Gesture gesture

float32 pitch
float32 yaw
float32 target_distance

float32 vel_x
float32 vel_y
float32 vel_yaw

# only compared to zero to determine the controller
# control_id > 0 : right
# control_id < 0 : left
float32 control_id
float32 spin_ctrl  # 1 顺 2 逆 3 stop