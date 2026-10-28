"""Bounded supervised navigation using measured odometry, never inferred full throttle."""
import importlib.util
import math
from pathlib import Path
from navigation_core import footprint_points, validate_calibration

def is_trial(config):
    return config.get('navigation_mode') == 'supervised_trial'

def validate_navigation(config):
    if not is_trial(config):
        return validate_calibration(config)
    # 2026-10-24: 上限从已验证包络(0.05/10%/15°/0.5)抬到实验包络
    # (0.12/20%/20°/0.7)。calibrated 仍为 false，这组数没有实车标定背书。
    # 2026-10-25: 标定前临时解除距离/角度限制（1m/1.2m/45s/20° -> 5m/6m/180s/25°）。
    # 传感器 0.4s 上限、净空 0.2m、速度 0.12m/s、PWM 20% 仍然生效。
    # Extended range is a selected supervised profile, not measured calibration.
    # It does not raise speed, duty, steering or sensor freshness limits.
    profile=config.get('supervised_profile','short')
    if profile not in ('short','range_10m'):
        raise ValueError('未知监督导航配置：'+str(profile))
    limits={'max_speed_mps':.28,'trial_pwm_cap':.45,'trial_max_steering_rad':math.radians(25),
        'trial_max_joystick':.85,'trial_max_duration_s':180.,'trial_max_goal_m':5.,
        'trial_max_travel_m':6.,'trial_max_path_m':6.}
    if profile=='range_10m':
        limits.update(trial_max_goal_m=10.,trial_max_travel_m=15.,
                      trial_max_path_m=15.,trial_max_duration_s=300.)
    for key,maximum in limits.items():
        value=config.get(key)
        if type(value) not in (float,int) or not math.isfinite(value) or not 0<value<=maximum+1e-9:
            raise ValueError('监督导航参数超出所选配置上限：'+key)
    for key in ('wheelbase_m','command_timeout_s','scan_timeout_s','odom_timeout_s','imu_timeout_s'):
        value=config.get(key)
        if type(value) not in (float,int) or not math.isfinite(value) or value<=0:
            raise ValueError('低速试航缺少有效参数：'+key)
    if config['scan_timeout_s']>.4 or config['odom_timeout_s']>.4 or config['imu_timeout_s']>.4:
        raise ValueError('试航传感器超时上限为0.4秒')
    if config.get('steering_sign') not in (-1,1):raise ValueError('转向符号无效')
    if type(config.get('trial_clearance_m')) not in (float,int) or not .2<=config['trial_clearance_m']<=.5:
        raise ValueError('试航净空应为0.2至0.5米')
    footprint_points(config)

class TrialDrive:
    def __init__(self, config):
        validate_navigation(config)
        if not is_trial(config):raise ValueError('Expected supervised trial profile')
        # Reuse the exact speed controller used in the physical 6.5cm/95cm runs.
        source=Path(__file__).resolve().parent.parent/'stage1/core.py'
        spec=importlib.util.spec_from_file_location('_rhino_stage1_feedback',source)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        self.feedback=module.Feedback(1/3,duty_cap=config['trial_pwm_cap'])
        self.config=config
    def reset(self):self.feedback.reset();self.over=0
    def step(self,v,w,measured_v,dt):
        # 0.15s 是「20Hz 控制节拍」的假设，但控制循环跑在网页广播循环里，
        # load 高时一次 emit 就能超过它。Feedback 内部本来就把 dt 钳到 0.04，
        # 所以这里只兜「循环真的冻住」，正常抖动交给它自己积分。
        if not all(math.isfinite(x) for x in (v,w,measured_v,dt)):
            raise ValueError('速度反馈或控制节拍异常')
        if not 0 < dt <= .5:
            # 2026-10-26: 进程停顿（VLM 推理等）会让 dt 变成数秒。
            # 这不是控制故障 —— 重置反馈、这一帧输出 0，下一帧照常。
            self.reset()
            return 0., 0., 0.
        if v<0 or v>self.config['max_speed_mps']+.001:raise ValueError('试航仅允许0至%.2f m/s前进'%self.config['max_speed_mps'])
        # 2026-10-25: 单帧尖峰不中止；连续 3 帧超 0.60 m/s 才判异常。
        # 控制用的速度钳到 ±0.5，尖峰不直接进反馈环。
        if measured_v<-.03 or measured_v>.60:
            self.over=getattr(self,'over',0)+1
            if self.over>=3:raise ValueError('实测速度异常，已停止试航')
        else:
            self.over=0
        measured_v=max(-.5,min(.5,measured_v))
        if v<.005:
            # 2026-10-26: 大角度转向改成**原地旋转** —— 舵机回中、两轮反向。
            # pwm_proc 的 rot 通道只认这个字段，手动摇杆永远不发，所以手动驾驶不变。
            if abs(w) > .25:
                limit = self.config.get('rotation_w', .8)
                self.reset()
                return 0., 0., max(-1., min(1., w/limit))
            # 终点前控制器偶尔会请求很小的 v；宁可**停住**交给目标检查器判定，
            # 也不要因为一句限制把已经走完的试航判死。
            self.reset();return 0.,0.,0.
        # 2026-10-24: δ=atan(L·ω/v)。接近终点时控制器把 v 降到很低，同样的 ω
        # 折算出的 δ 会发散，最后几厘米被这条限制拦下。物理上舵机只能打到
        # maximum，所以**饱和执行**而不是中止；只有 v≈0 的原地转向才单独处理。
        steering=math.atan(self.config['wheelbase_m']*w/max(v,0.02))
        maximum=self.config['trial_max_steering_rad']
        if abs(steering)>maximum:
            steering=math.copysign(maximum,steering)
        l,_=self.feedback.step(min(v,self.config['max_speed_mps']),0.,measured_v,0.,dt)
        a=self.config['steering_sign']*max(-1.,min(1.,steering/maximum))*self.config['trial_max_joystick']
        return min(l,self.config['trial_pwm_cap']),a,0.

def scan_clearance(scan,mount,config,corridor_deg=70.0):
    """Nearest live return in the FORWARD corridor of the measured rectangle.

    2026-10-24: 原来算整个矩形四周的最近点，窄房间里侧后方 20cm 的物体就会
    把「净空」压到阈值以下、假报近处障碍。试航是低速直行，只关心前方
    ±corridor_deg 的走廊；侧向/后方由舵机 ±15° 的几何决定，不需要整车矩形。
    """
    footprint=footprint_points(config)
    front,rear,halfwidth=footprint[0][0],-footprint[2][0],footprint[0][1]
    c,s=math.cos(mount['yaw_rad']),math.sin(mount['yaw_rad'])
    distances=[]
    for index,value in enumerate(scan.ranges):
        if not math.isfinite(value) or not max(.05,scan.range_min)<value<scan.range_max:continue
        angle=scan.angle_min+index*scan.angle_increment
        px,py=value*math.cos(angle),value*math.sin(angle)
        x,y=c*px-s*py+mount['x_m'],s*px+c*py+mount['y_m']
        if math.hypot(x,y)>1e-6 and abs(math.atan2(y,x))>math.radians(corridor_deg):
            continue
        distances.append(math.hypot(max(-rear-x,0.,x-front),max(abs(y)-halfwidth,0.)))
    return min(distances) if len(distances)>=20 else None
