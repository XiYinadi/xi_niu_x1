"""A failed replan may reuse only this task's collision-checked route."""


def single_goal_tree(trial=False):
    if not trial:
        return '''<root main_tree_to_execute="MainTree">
<BehaviorTree ID="MainTree"><PipelineSequence name="Navigate">
<RateController hz="1.0"><ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/></RateController>
<FollowPath path="{path}" controller_id="FollowPath"/>
</PipelineSequence></BehaviorTree></root>
'''
    # Humble ComputePathToPose clears its output on abort/halt. Keep its
    # candidate separate so a failed attempt cannot erase the active route.
    # The initial plan is mandatory for every task, even inside the near band.
    # Route validity permits REUSE, never success: FollowPath still has to
    # satisfy the controller's unchanged 8 cm / 0.35 rad goal checker.
    return '''<root main_tree_to_execute="MainTree">
<BehaviorTree ID="MainTree">
<Sequence name="RequireFreshInitialRoute">
  <ComputePathToPose goal="{goal}" path="{active_path}" planner_id="GridBased"/>
  <PipelineSequence name="Navigate">
    <RateController hz="1.0">
      <Fallback name="ReplanOrFollowValidTaskRoute">
        <Sequence name="ReplaceOnlyOnSuccessfulPlan">
          <ComputePathToPose goal="{goal}" path="{candidate_path}" planner_id="GridBased"/>
          <SetBlackboard output_key="active_path" value="{candidate_path}"/>
        </Sequence>
        <IsPathValid path="{active_path}" server_timeout="500"/>
      </Fallback>
    </RateController>
    <FollowPath path="{active_path}" controller_id="FollowPath"/>
  </PipelineSequence>
</Sequence>
</BehaviorTree></root>
'''
