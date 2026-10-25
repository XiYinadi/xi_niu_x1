"""Pure motion conversion and freshness gates for a servo-steered chassis."""
import math


def footprint_points(config):
    """Rectangle in base_link axes, including a rear-axle reference if measured."""
    length,width=config.get('length_m') or .7,config.get('width_m') or .7
    front=rear=length/2
    if config.get('base_reference') == 'rear_axle_midpoint':
        front,rear=config.get('front_extent_m'),config.get('rear_extent_m')
    for value in (length,width,front,rear):
        if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
            raise ValueError('车体轮廓必须包含有效的前伸、后伸和宽度')
    if not math.isclose(front+rear,length,rel_tol=1e-6,abs_tol=1e-6):
        raise ValueError('车体前伸与后伸之和必须等于整车长度')
    return [[front,width/2],[front,-width/2],[-rear,-width/2],[-rear,width/2]]


def validate_calibration(config):
    if config.get('calibrated') is not True:
        raise ValueError('底盘尺寸、转角与速度尚未实测标定；当前只允许定位和路径预览')
    for key in ('wheelbase_m', 'length_m', 'width_m', 'max_steering_rad',
                'full_throttle_mps', 'max_speed_mps', 'command_timeout_s',
                'scan_timeout_s', 'localization_timeout_s'):
        v = config.get(key)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            raise ValueError(f'{key} 必须是已实测的正数')
    if config['max_steering_rad'] >= math.pi/2:
        raise ValueError('最大转角必须小于 90 度')
    if config.get('steering_sign') not in (-1, 1):
        raise ValueError('steering_sign 必须是 -1 或 1')
    if config['max_speed_mps'] > config['full_throttle_mps']:
        raise ValueError('导航限速不能超过标定满油门速度')
    footprint_points(config)


def twist_to_joystick(v, omega, config):
    validate_calibration(config)
    if not all(math.isfinite(x) for x in (v, omega)):
        raise ValueError('导航速度不是有限数值')
    if abs(v) < 0.005:
        if abs(omega) > 0.01:
            raise ValueError('舵机转向底盘不支持原地旋转')
        return 0.0, 0.0
    if v < 0:
        raise ValueError('第一版导航不允许倒车')
    # Preserve curvature when limiting speed, rather than increasing steering.
    steering = math.atan(config['wheelbase_m'] * omega / v)
    if abs(steering) > config['max_steering_rad'] + 0.01:
        raise ValueError('导航曲率超过实测底盘转向能力')
    throttle = min(v, config['max_speed_mps']) / config['full_throttle_mps']
    a = config['steering_sign'] * steering / config['max_steering_rad']
    return throttle, max(-1.0, min(1.0, a))


def fresh(now, then, timeout):
    return then is not None and 0 <= now-then < timeout
