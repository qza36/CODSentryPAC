# 多项式轨迹消息
# 每段轨迹用4阶多项式表示: p(t) = c0 + c1*t + c2*t^2 + c3*t^3

builtin_interfaces/Time start_time  # 轨迹起始时间
uint8 motion_mode                   # 运动模式
float32[] coef_x                    # X方向多项式系数 (每段4个系数)
float32[] coef_y                    # Y方向多项式系数 (每段4个系数)
float32[] duration                  # 每段轨迹的时间长度
