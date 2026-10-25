"""Start one fresh Nav2 lifecycle group after DDS discovery has settled.

The Humble manager creates service clients and immediately autostarts. A lost
change_state response can leave it waiting forever although the node changed
state. Pre-create the manager with autostart=false, let its clients discover the
server response endpoints, then request STARTUP once. No motion publishers.
"""
import argparse
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manager',required=True,choices=['lifecycle_manager_localization','lifecycle_manager_navigation'])
    parser.add_argument('--nodes',required=True,nargs='+',choices=['map_server','amcl','controller_server','planner_server','bt_navigator','velocity_smoother'])
    args=parser.parse_args()
    import rclpy
    from nav2_msgs.srv import ManageLifecycleNodes
    from lifecycle_msgs.srv import GetState
    from std_srvs.srv import Trigger
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    rclpy.init();node=rclpy.create_node(args.manager+'_startup')
    buffer=Buffer();listener=TransformListener(buffer,node)
    startup=node.create_client(ManageLifecycleNodes,'/'+args.manager+'/manage_nodes')
    active=node.create_client(Trigger,'/'+args.manager+'/is_active')
    states={name:node.create_client(GetState,'/'+name+'/get_state') for name in args.nodes}

    def spin_until(predicate,seconds,label):
        deadline=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<deadline:
            if predicate():return
            rclpy.spin_once(node,timeout_sec=.05)
        raise RuntimeError('Timed out: '+label)

    def reply(client,request,label,seconds=8):
        future=client.call_async(request)
        try:
            spin_until(future.done,seconds,label)
            return future.result()
        except Exception:
            client.remove_pending_request(future);future.cancel()
            raise

    try:
        if args.manager=='lifecycle_manager_navigation':
            # Planner activation blocks inside costmap until map TF exists.
            # Let AMCL start independently and wait for the operator's real
            # initial pose BEFORE asking the manager to activate navigation.
            node.get_logger().info('Waiting for fresh map->base_link and odom->base_link before navigation startup; set initial pose')
            while rclpy.ok():
                rclpy.spin_once(node,timeout_sec=.1)
                try:
                    transforms=[buffer.lookup_transform(frame,'base_link',Time()) for frame in ('map','odom')]
                    now=node.get_clock().now().nanoseconds/1e9
                    if all(-.05<now-(tf.header.stamp.sec+tf.header.stamp.nanosec/1e9)<.5 for tf in transforms):
                        break
                except Exception:
                    pass
            if not rclpy.ok():return 1
        clients=[startup,active,*states.values()]
        spin_until(lambda:all(c.service_is_ready() for c in clients),20,'lifecycle service discovery')
        # Both manager clients and this helper's response readers must match.
        settled=time.monotonic()+2
        spin_until(lambda:time.monotonic()>=settled,3,'discovery settling')
        initial={name:reply(c,GetState.Request(),name+' initial state').current_state.id for name,c in states.items()}
        if any(value!=1 for value in initial.values()):
            raise RuntimeError('Expected a fresh unconfigured group; will not replay STARTUP: '+str(initial))
        request=ManageLifecycleNodes.Request();request.command=ManageLifecycleNodes.Request.STARTUP
        result=reply(startup,request,args.manager+' STARTUP',25)
        if not result.success:raise RuntimeError('Lifecycle manager rejected startup')
        final={name:reply(c,GetState.Request(),name+' final state').current_state.id for name,c in states.items()}
        if any(value!=3 for value in final.values()) or not reply(active,Trigger.Request(),'manager active').success:
            raise RuntimeError('Lifecycle group not fully active: '+str(final))
        node.get_logger().info('Verified lifecycle group ACTIVE: '+str(final))
        return 0
    except Exception as error:
        node.get_logger().error('Navigation startup incomplete; motion remains gated: '+str(error))
        return 1
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':sys.exit(main())
