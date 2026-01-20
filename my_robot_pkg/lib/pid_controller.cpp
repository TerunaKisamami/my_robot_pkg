// PIDのPの部分
float calc_pos_p(float target, float current, const float kp) {
  float error = target - current;
  float pos_P = kp + error;

  return pos_P;
}
// PIDのIの部分
float calc_pos_i(float target, float current, float dt, float pre_i_value,
                 float ki, const int max_i_value) {
  float error = target - current;
  float pos_I = pre_i_value + ki * error * dt;

  // 上下限を超えたときは調整
  if (pos_I > max_i_value) {
    pos_I = max_i_value;
  }
  if (pos_I < -max_i_value) {
    pos_I = -max_i_value;
  }

  return pos_I;
}
// PIDのDの部分
float calc_pos_d(float target, float current, float dt, float pre_error,
                 const float kd) {
  float error = target - current;
  float pos_D = kd * (error - pre_error) / dt;

  return pos_D;
}

float calc_vel_p(float error, float pre_error, const float kp) {
  float vel_P = kp * (error - pre_error);
  return vel_P;
}

float calc_vel_i(float error, float dt, const float ki) {
  float vel_I = ki * error * dt;
  return vel_I;
}
float calc_vel_d(float error, float pre_error, float pre_pre_error,
                 const float kd) {
  float vel_d = kd * (error + pre_pre_error - 2 * pre_error);
  return vel_d;
}
