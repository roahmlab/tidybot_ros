#!/usr/bin/env python3
"""
Drawer Policy Node

This node provides a service interface for receiving drawer task specifications
and compiles them into multi-stage motion sequences that are sent to the
generic multi_stage_executor.

Service: /open_drawer_task (OpenDrawerTask.srv)
Action Client: /execute_stages (ExecuteStages.action)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from tidybot_utils.srv import OpenDrawerTask
from tidybot_utils.action import ExecuteStages
from tidybot_utils.msg import MotionStage

from geometry_msgs.msg import Pose, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty as EmptySrv, SetBool

import numpy as np
from scipy.spatial.transform import Rotation as R
import math


class DrawerPolicyNode(Node):
    """
    High-level drawer task policy server.
    
    Receives drawer specifications via service, compiles them into
    MotionStage sequences, and executes them via the multi_stage_executor.
    """
    
    def __init__(self):
        super().__init__('drawer_policy')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Service server for receiving drawer task requests
        self.drawer_service = self.create_service(
            OpenDrawerTask,
            'open_drawer_task',
            self.handle_drawer_request,
            callback_group=self.callback_group
        )
        
        # Action client for executing stages
        self.executor_client = ActionClient(
            self, 
            ExecuteStages, 
            'execute_stages',
            callback_group=self.callback_group
        )
        
        # Publisher for RViz visualization
        self.marker_pub = self.create_publisher(MarkerArray, 'drawer_task_markers', 10)
        
        # Parameters
        self.declare_parameter('approach_distance', 0.15)  # Distance to approach from
        self.declare_parameter('approach_duration', 3.0)   # PTP motion duration
        self.declare_parameter('grasp_duration', 2.0)      # LIN motion duration
        self.declare_parameter('pull_duration', 4.0)       # LIN/CIRC motion duration
        self.declare_parameter('gripper_duration', 3.0)    # Gripper action duration
        self.declare_parameter('record_sensor_data', True)  # Enable sensor recording
        
        # Sensor data recorder service clients
        self.record_enabled = self.get_parameter('record_sensor_data').value
        if self.record_enabled:
            self.recorder_start_client = self.create_client(
                EmptySrv, '/sensor_recorder/start',
                callback_group=self.callback_group
            )
            self.recorder_stop_client = self.create_client(
                EmptySrv, '/sensor_recorder/stop',
                callback_group=self.callback_group
            )
            self.recorder_save_client = self.create_client(
                SetBool, '/sensor_recorder/save',
                callback_group=self.callback_group
            )
        
        self.get_logger().info('Drawer Policy Node initialized')
        self.get_logger().info('  Service: /open_drawer_task')
        self.get_logger().info('  Action client: /execute_stages')
        self.get_logger().info(f'  Sensor recording: {"enabled" if self.record_enabled else "disabled"}')

    async def _call_recorder_start(self):
        """Start sensor data recording."""
        if not self.record_enabled:
            return
        if not self.recorder_start_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Sensor recorder start service not available')
            return
        await self.recorder_start_client.call_async(EmptySrv.Request())
        self.get_logger().info('Sensor recording started')

    async def _call_recorder_stop(self):
        """Stop sensor data recording."""
        if not self.record_enabled:
            return
        if not self.recorder_stop_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Sensor recorder stop service not available')
            return
        await self.recorder_stop_client.call_async(EmptySrv.Request())
        self.get_logger().info('Sensor recording stopped')

    async def _call_recorder_save(self, save: bool):
        """Save (True) or discard (False) recorded sensor data."""
        if not self.record_enabled:
            return
        if not self.recorder_save_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Sensor recorder save service not available')
            return
        req = SetBool.Request()
        req.data = save
        result = await self.recorder_save_client.call_async(req)
        action = 'saved' if save else 'discarded'
        self.get_logger().info(f'Sensor data {action}: {result.message}')

    async def handle_drawer_request(self, request, response):
        """
        Handle incoming drawer task request.
        Compiles the request into MotionStage sequences and executes them.
        Controls sensor data recording: start before execution, save/discard after.
        """
        self.get_logger().info(f'Received drawer task request: joint_type={request.joint_type}')
        
        # Validate request
        if request.joint_type not in ['prismatic', 'revolute']:
            response.accepted = False
            response.message = f"Invalid joint_type: {request.joint_type}"
            return response
        
        # Compile drawer specs into motion stages
        try:
            stages = self.compile_drawer_stages(
                joint_type=request.joint_type,
                handle_pose=request.handle_pose,
                joint_axis_origin=request.joint_axis_origin,
                joint_axis=request.joint_axis,
                pull_amount=request.pull_amount
            )
        except Exception as e:
            response.accepted = False
            response.message = f"Failed to compile stages: {str(e)}"
            return response
        
        # Publish visualization markers
        self.publish_markers(request, stages)
        
        # Wait for executor action server
        if not self.executor_client.wait_for_server(timeout_sec=5.0):
            response.accepted = False
            response.message = "Executor action server not available"
            return response
        
        # Start sensor data recording
        await self._call_recorder_start()
        
        # Send goal to executor
        goal = ExecuteStages.Goal()
        goal.stages = stages
        
        self.get_logger().info(f'Sending {len(stages)} stages to executor')
        
        # Send goal and wait for result
        future = self.executor_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        
        # Wait for the goal to be accepted
        try:
            goal_handle = await future
        except Exception as e:
            # Stop recording and discard on failure
            await self._call_recorder_stop()
            await self._call_recorder_save(save=False)
            response.accepted = False
            response.message = f"Action call failed: {str(e)}"
            return response
        
        if not goal_handle.accepted:
            # Stop recording and discard on rejection
            await self._call_recorder_stop()
            await self._call_recorder_save(save=False)
            response.accepted = False
            response.message = "Goal rejected by executor"
            return response
            
        self.get_logger().info("Goal accepted by executor, waiting for result...")
        
        # Wait for the result
        result_future = goal_handle.get_result_async()
        
        try:
            result_wrapper = await result_future
            result = result_wrapper.result
            
            # Stop recording
            await self._call_recorder_stop()
            
            if result.success:
                # Save recorded data on success
                await self._call_recorder_save(save=True)
                response.accepted = True
                response.message = f"Task completed successfully: {result.message}"
            else:
                # Discard recorded data on failure
                await self._call_recorder_save(save=False)
                response.accepted = False
                response.message = f"Task failed: {result.message}"
        except Exception as e:
            # Stop recording and discard on error
            await self._call_recorder_stop()
            await self._call_recorder_save(save=False)
            response.accepted = False
            response.message = f"Failed to get result: {str(e)}"
            
        return response

    def feedback_callback(self, feedback_msg):
        """Log feedback from executor."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Stage {feedback.current_stage_index}: {feedback.current_stage_description} '
            f'({feedback.stage_progress:.0%})'
        )

    def compile_drawer_stages(self, joint_type, handle_pose, joint_axis_origin, 
                              joint_axis, pull_amount):
        """
        Compile drawer specifications into MotionStage sequence.
        
        Args:
            joint_type: "prismatic" or "revolute"
            handle_pose: geometry_msgs/Pose - handle position and grasp orientation
            joint_axis_origin: geometry_msgs/Point - pivot point for revolute
            joint_axis: geometry_msgs/Vector3 - unit vector for joint direction
            pull_amount: float - distance (m) for prismatic, angle (rad) for revolute
        
        Returns:
            list of MotionStage messages
        """
        stages = []
        
        # Get parameters
        approach_dist = self.get_parameter('approach_distance').value
        approach_dur = self.get_parameter('approach_duration').value
        grasp_dur = self.get_parameter('grasp_duration').value
        pull_dur = self.get_parameter('pull_duration').value
        gripper_dur = self.get_parameter('gripper_duration').value
        
        # Calculate poses
        # Convert handle orientation to rotation matrix
        quat = [handle_pose.orientation.x, handle_pose.orientation.y,
                handle_pose.orientation.z, handle_pose.orientation.w]
        rot = R.from_quat(quat)
        rot_matrix = rot.as_matrix()
        
        # Approach pose: offset along -Z_tool direction (backing off from grasp)
        z_tool = rot_matrix[:, 2]  # Z-axis of tool frame in world
        approach_offset = -z_tool * approach_dist
        
        approach_pose = Pose()
        approach_pose.position.x = handle_pose.position.x + approach_offset[0]
        approach_pose.position.y = handle_pose.position.y + approach_offset[1]
        approach_pose.position.z = handle_pose.position.z + approach_offset[2]
        approach_pose.orientation = handle_pose.orientation
        
        # Calculate pull pose based on joint type
        if joint_type == 'prismatic':
            pull_pose = self._calculate_prismatic_pull(
                handle_pose, joint_axis, pull_amount
            )
        else:  # revolute
            pull_pose = self._calculate_revolute_pull(
                handle_pose, joint_axis_origin, joint_axis, pull_amount
            )
        
        # Stage 1: Open gripper
        stage1 = MotionStage()
        stage1.stage_type = MotionStage.STAGE_GRIPPER
        stage1.gripper_position = 0.0  # Open
        stage1.duration = gripper_dur
        stage1.description = "Open gripper"
        stages.append(stage1)
        
        # Stage 2: PTP to approach pose
        stage2 = MotionStage()
        stage2.stage_type = MotionStage.STAGE_PTP
        stage2.target_pose = approach_pose
        stage2.duration = approach_dur
        stage2.velocity_scaling = 0.2
        stage2.description = "Approach handle"
        stages.append(stage2)
        
        # Stage 3: LIN to grasp pose
        stage3 = MotionStage()
        stage3.stage_type = MotionStage.STAGE_LIN
        stage3.target_pose = handle_pose
        stage3.duration = grasp_dur
        stage3.velocity_scaling = 0.1
        stage3.description = "Move to grasp"
        stages.append(stage3)
        
        # Stage 4: Close gripper
        stage4 = MotionStage()
        stage4.stage_type = MotionStage.STAGE_GRIPPER
        stage4.gripper_position = 1.0  # Closed (scaled to 0.8 by executor)
        stage4.duration = gripper_dur * 2  # Extra time for gripper to close
        stage4.description = "Close gripper"
        stages.append(stage4)
        
        # Stage 5: Pull motion (LIN for prismatic, CIRC for revolute)
        stage5 = MotionStage()
        if joint_type == 'prismatic':
            stage5.stage_type = MotionStage.STAGE_LIN
            stage5.target_pose = pull_pose
        else:  # revolute
            stage5.stage_type = MotionStage.STAGE_CIRC
            stage5.target_pose = pull_pose
            stage5.arc_center = joint_axis_origin
            stage5.arc_axis.x = joint_axis.x
            stage5.arc_axis.y = joint_axis.y
            stage5.arc_axis.z = joint_axis.z
            stage5.arc_angle = pull_amount
        stage5.duration = pull_dur
        stage5.velocity_scaling = 0.1
        stage5.description = "Pull drawer"
        stages.append(stage5)
        
        return stages

    def _calculate_prismatic_pull(self, handle_pose, joint_axis, pull_amount):
        """
        Calculate pull pose for prismatic drawer.
        
        Positive pull_amount: move along +joint_axis direction
        Negative pull_amount: move along -joint_axis direction
        """
        pull_pose = Pose()
        
        # Normalize joint axis
        axis = np.array([joint_axis.x, joint_axis.y, joint_axis.z])
        axis_norm = np.linalg.norm(axis)
        if axis_norm > 0:
            axis = axis / axis_norm
        
        # Calculate offset
        offset = axis * pull_amount
        
        pull_pose.position.x = handle_pose.position.x + offset[0]
        pull_pose.position.y = handle_pose.position.y + offset[1]
        pull_pose.position.z = handle_pose.position.z + offset[2]
        pull_pose.orientation = handle_pose.orientation
        
        return pull_pose

    def _calculate_revolute_pull(self, handle_pose, axis_origin, joint_axis, pull_angle):
        """
        Calculate pull pose for revolute door.
        
        Rotates handle position around the axis_origin by pull_angle.
        Positive angle: CCW around joint_axis (right-hand rule)
        """
        pull_pose = Pose()
        
        # Handle position relative to pivot
        handle_pos = np.array([
            handle_pose.position.x,
            handle_pose.position.y,
            handle_pose.position.z
        ])
        pivot = np.array([axis_origin.x, axis_origin.y, axis_origin.z])
        rel_pos = handle_pos - pivot
        
        # Rotation around joint axis
        axis = np.array([joint_axis.x, joint_axis.y, joint_axis.z])
        rot = R.from_rotvec(axis * pull_angle)
        
        # Rotated position
        new_rel_pos = rot.apply(rel_pos)
        new_pos = pivot + new_rel_pos
        
        pull_pose.position.x = new_pos[0]
        pull_pose.position.y = new_pos[1]
        pull_pose.position.z = new_pos[2]
        
        # Also rotate the gripper orientation
        handle_quat = [
            handle_pose.orientation.x,
            handle_pose.orientation.y,
            handle_pose.orientation.z,
            handle_pose.orientation.w
        ]
        handle_rot = R.from_quat(handle_quat)
        new_rot = rot * handle_rot
        new_quat = new_rot.as_quat()
        
        pull_pose.orientation.x = new_quat[0]
        pull_pose.orientation.y = new_quat[1]
        pull_pose.orientation.z = new_quat[2]
        pull_pose.orientation.w = new_quat[3]
        
        return pull_pose

    def publish_markers(self, request, stages):
        """
        Publish RViz markers for visualization.
        
        Shows:
        - Handle position (green sphere)
        - Grasp orientation (RGB frame axes)
        - Pulling trajectory (white curve)
        - End position (red sphere)
        """
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        marker_id = 0
        
        # Delete all previous markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)
        
        # Get handle and pull poses from stages
        handle_pose = request.handle_pose
        pull_pose = None
        approach_pose = None
        
        for stage in stages:
            if stage.description == "Pull drawer":
                pull_pose = stage.target_pose
            elif stage.description == "Approach handle":
                approach_pose = stage.target_pose
        
        # 1. Handle position (GREEN sphere)
        handle_marker = Marker()
        handle_marker.header.frame_id = "world"
        handle_marker.header.stamp = stamp
        handle_marker.ns = "drawer_viz"
        handle_marker.id = marker_id
        marker_id += 1
        handle_marker.type = Marker.SPHERE
        handle_marker.action = Marker.ADD
        handle_marker.pose = handle_pose
        handle_marker.scale.x = 0.03
        handle_marker.scale.y = 0.03
        handle_marker.scale.z = 0.03
        handle_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        markers.markers.append(handle_marker)
        
        # 2. Grasp orientation (RGB frame axes)
        quat = [handle_pose.orientation.x, handle_pose.orientation.y,
                handle_pose.orientation.z, handle_pose.orientation.w]
        rot = R.from_quat(quat)
        rot_matrix = rot.as_matrix()
        
        axis_colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # X - Red
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # Y - Green
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # Z - Blue
        ]
        axis_length = 0.08
        
        for i, color in enumerate(axis_colors):
            arrow = Marker()
            arrow.header.frame_id = "world"
            arrow.header.stamp = stamp
            arrow.ns = "drawer_frame"
            arrow.id = marker_id
            marker_id += 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            
            origin = Point(
                x=handle_pose.position.x,
                y=handle_pose.position.y,
                z=handle_pose.position.z
            )
            axis_dir = rot_matrix[:, i] * axis_length
            end_point = Point(
                x=handle_pose.position.x + axis_dir[0],
                y=handle_pose.position.y + axis_dir[1],
                z=handle_pose.position.z + axis_dir[2]
            )
            
            arrow.points.append(origin)
            arrow.points.append(end_point)
            arrow.scale.x = 0.008  # shaft diameter
            arrow.scale.y = 0.015  # head diameter
            arrow.scale.z = 0.015  # head length
            arrow.color = color
            markers.markers.append(arrow)
        
        # 3. Trajectory curve (WHITE line strip)
        if pull_pose is not None:
            trajectory = Marker()
            trajectory.header.frame_id = "world"
            trajectory.header.stamp = stamp
            trajectory.ns = "drawer_trajectory"
            trajectory.id = marker_id
            marker_id += 1
            trajectory.type = Marker.LINE_STRIP
            trajectory.action = Marker.ADD
            trajectory.scale.x = 0.008  # line width
            trajectory.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            
            # Add trajectory points
            if approach_pose is not None:
                trajectory.points.append(Point(
                    x=approach_pose.position.x,
                    y=approach_pose.position.y,
                    z=approach_pose.position.z
                ))
            
            trajectory.points.append(Point(
                x=handle_pose.position.x,
                y=handle_pose.position.y,
                z=handle_pose.position.z
            ))
            
            # For revolute, interpolate arc; for prismatic, straight line
            if request.joint_type == "revolute":
                # Interpolate arc points
                num_arc_points = 10
                axis = np.array([request.joint_axis.x, request.joint_axis.y, request.joint_axis.z])
                pivot = np.array([request.joint_axis_origin.x, request.joint_axis_origin.y, 
                                  request.joint_axis_origin.z])
                handle_pos = np.array([handle_pose.position.x, handle_pose.position.y, 
                                       handle_pose.position.z])
                rel_pos = handle_pos - pivot
                
                for j in range(1, num_arc_points + 1):
                    frac = j / num_arc_points
                    angle = request.pull_amount * frac
                    rot_arc = R.from_rotvec(axis * angle)
                    arc_pos = pivot + rot_arc.apply(rel_pos)
                    trajectory.points.append(Point(x=arc_pos[0], y=arc_pos[1], z=arc_pos[2]))
            else:
                # Prismatic: straight line to pull pose
                trajectory.points.append(Point(
                    x=pull_pose.position.x,
                    y=pull_pose.position.y,
                    z=pull_pose.position.z
                ))
            
            markers.markers.append(trajectory)
        
        # 4. End position (RED sphere)
        if pull_pose is not None:
            end_marker = Marker()
            end_marker.header.frame_id = "world"
            end_marker.header.stamp = stamp
            end_marker.ns = "drawer_viz"
            end_marker.id = marker_id
            marker_id += 1
            end_marker.type = Marker.SPHERE
            end_marker.action = Marker.ADD
            end_marker.pose = pull_pose
            end_marker.scale.x = 0.03
            end_marker.scale.y = 0.03
            end_marker.scale.z = 0.03
            end_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            markers.markers.append(end_marker)
        
        self.marker_pub.publish(markers)
        self.get_logger().info(f'Published {len(markers.markers)} visualization markers')


def main(args=None):
    rclpy.init(args=args)
    node = DrawerPolicyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
