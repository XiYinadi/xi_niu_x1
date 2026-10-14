"""Pure motion conversion and freshness gates for a servo-steered chassis."""
import math


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
