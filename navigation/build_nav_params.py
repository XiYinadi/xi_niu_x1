"""Derive Humble-specific defaults from the installed Nav2 distribution."""
import math
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from navigation_core import footprint_points
from navigation_drive import is_trial, validate_navigation


def build():
    here = Path(__file__).resolve().parent
    cfg = yaml.safe_load((here/'navigation_calibration.yaml').read_text())
    trial=is_trial(cfg)
    if trial:validate_navigation(cfg)
    share = Path(get_package_share_directory('nav2_bringup'))
    data = yaml.safe_load((share/'params'/'nav2_params.yaml').read_text())
    for section in data.values():
        if isinstance(section, dict) and 'ros__parameters' in section:
            section['ros__parameters']['use_sim_time'] = False
    amcl = data['amcl']['ros__parameters']
    amcl.update(base_frame_id='base_link', odom_frame_id='odom', global_frame_id='map',
                robot_model_type='nav2_amcl::DifferentialMotionModel',
                scan_topic='/scan', set_initial_pose=False,
                update_min_d=0.02, update_min_a=0.03, transform_tolerance=0.5,
                max_beams=35, laser_max_range=8.0,
                recovery_alpha_slow=0.001, recovery_alpha_fast=0.1)
    # DifferentialMotionModel models odometry noise, not drive commands.
    # Reference changes must move the footprint as well as the laser TF.
    footprint = str(footprint_points(cfg))
    for key in ('local_costmap', 'global_costmap'):
        p = data[key][key]['ros__parameters']
        p.update(use_sim_time=False, robot_base_frame='base_link', footprint=footprint, resolution=0.05,
                 transform_tolerance=0.5)
        p.pop('robot_radius', None)
        # A 2D laser cannot populate a useful volumetric voxel layer here.
        p['plugins'] = (['static_layer'] if key == 'global_costmap' else []) + ['obstacle_layer', 'inflation_layer']
        p['obstacle_layer'] = dict(plugin='nav2_costmap_2d::ObstacleLayer', enabled=True,
            observation_sources='scan', scan=dict(topic='/scan', data_type='LaserScan',
            min_obstacle_height=0.0, max_obstacle_height=2.0,
            marking=True, clearing=True, obstacle_min_range=0.2, obstacle_max_range=3.0,
            raytrace_min_range=0.2, raytrace_max_range=4.0))
        # 2026-10-26: 膨胀 0.45 -> 0.35、代价斜率 3.0 -> 4.0，避免窄处被逼着绕远路
        p['inflation_layer'].update(inflation_radius=0.30, cost_scaling_factor=5.0)
        if key == 'global_costmap':
            p['static_layer'].update(map_subscribe_transient_local=True,subscribe_to_updates=True)
    planning_steering=cfg['trial_max_steering_rad']*.95 if trial else (cfg.get('max_steering_rad') or .4)
    # 2026-10-25: 理论半径（轮距/tan 舵角）比实车小，允许配置直接覆盖。
    radius = float(cfg.get('planning_turning_radius_m') or 0.0) or \
        (cfg.get('wheelbase_m') or 0.4) / math.tan(planning_steering)
    planner = data['planner_server']['ros__parameters']
    planner['planner_plugins'] = ['GridBased']
    planner['GridBased'] = dict(plugin='nav2_smac_planner/SmacPlannerHybrid',
        # 2026-10-27: 改回 true —— 关掉之后 zou_lang 这种有大片未知区的图直接
        # 规划失败（status=6，预览报「第 1 段不可达」）。绕圈问题已经由短腿直行解决。
        tolerance=0.15, downsample_costmap=False, allow_unknown=True,
        max_iterations=200000, max_planning_time=5.0, motion_model_for_search='DUBIN',
        angle_quantization_bins=72, minimum_turning_radius=radius,
        analytic_expansion_ratio=3.5, analytic_expansion_max_length=max(3.0, 4*radius))
    controller = data['controller_server']['ros__parameters']
    controller.update(controller_frequency=15.0, odom_topic='/odom', controller_plugins=['FollowPath'])
    controller['FollowPath'] = dict(plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        desired_linear_vel=cfg['max_speed_mps'], lookahead_dist=0.4,
        min_lookahead_dist=0.25, max_lookahead_dist=0.6, lookahead_time=1.5,
        use_velocity_scaled_lookahead_dist=True, use_rotate_to_heading=False,
        allow_reversing=False, use_collision_detection=True,
        max_allowed_time_to_collision_up_to_carrot=1.0,
        use_regulated_linear_velocity_scaling=True, use_cost_regulated_linear_velocity_scaling=True,
        regulated_linear_scaling_min_radius=radius,
        regulated_linear_scaling_min_speed=0.03, min_approach_linear_velocity=0.02,
        transform_tolerance=0.3)
    controller['general_goal_checker']['xy_goal_tolerance'] = 0.15
    controller['general_goal_checker']['yaw_goal_tolerance'] = 0.35
    if trial:
        controller['general_goal_checker']['xy_goal_tolerance']=.08
        # Default 0.5m/10s cannot be met by this 0.05m/s profile at startup.
        controller['progress_checker'].update(required_movement_radius=.03,movement_time_allowance=8.0)
        smoother=data['velocity_smoother']['ros__parameters']
        max_w=cfg['max_speed_mps']*math.tan(cfg['trial_max_steering_rad'])/cfg['wheelbase_m']
        smoother.update(smoothing_frequency=20.0,feedback='OPEN_LOOP',scale_velocities=True,
            max_velocity=[cfg['max_speed_mps'],0.,max_w],min_velocity=[0.,0.,-max_w],
            max_accel=[.04,0.,.10],max_decel=[-.08,0.,-.15],velocity_timeout=.2)
    # No spin/backup recovery: both assume capabilities not established on this chassis.
    bt = here/'navigate_no_recovery.xml'
    from navigation_tree import single_goal_tree
    bt.write_text(single_goal_tree(trial))
    data['bt_navigator']['ros__parameters'].update(
        robot_base_frame='base_link', global_frame='map', odom_topic='/odom',
        # Humble's 20ms action ACK default intermittently expires under load.
        # Milliseconds; independent of the actuator/sensor stop watchdogs.
        default_server_timeout=500,
        default_nav_to_pose_bt_xml=str(bt))
    multi = here/'navigate_through_no_recovery.xml'
    multi.write_text('''<root main_tree_to_execute="MainTree">
<BehaviorTree ID="MainTree"><PipelineSequence name="Navigate">
<RateController hz="1.0"><ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="GridBased"/></RateController>
<FollowPath path="{path}" controller_id="FollowPath"/>
</PipelineSequence></BehaviorTree></root>\n''')
    data['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml'] = str(multi)
    (here/'generated_params.yaml').write_text(yaml.safe_dump(data, sort_keys=False))


if __name__ == '__main__':
    build()
