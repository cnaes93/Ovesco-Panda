#! /usr/bin/env python3
import franka_gripper.msg
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import DisplayTrajectory
# Brings in the SimpleActionClient
import actionlib




try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True



def go_home(v=0.1,a=0.08):
        #set the Robot in Home Position
    global move_group 
    move_group = moveit_commander.MoveGroupCommander("panda_arm")
    move_group.set_planning_pipeline_id('pilz_industrial_motion_planner')
    move_group.set_planner_id('LIN')
    move_group.set_max_velocity_scaling_factor(v)
    move_group.set_max_acceleration_scaling_factor(a)
    global HomePos
    HomePos= move_group.get_current_pose().pose
    HomePos.position.x    =  0.3069401023282745 
    HomePos.position.y    =  -3.119671042482737e-05
    HomePos.position.z    =  0.5905445589541893
    HomePos.orientation.x =  -0.92385647535886
    HomePos.orientation.y =  0.38273843964818177
    HomePos.orientation.z =  -0.0006528054373945686
    HomePos.orientation.w =  0.000271289955921811
   

    move_group.set_pose_target(HomePos)
    success = move_group.go(wait=True)
   
        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
    move_group.clear_pose_targets()
    current_pose = move_group.get_current_pose().pose
    return all_close(HomePos, current_pose, 0.01)


def open_gripper():
    global gripper 
    gripper = actionlib.SimpleActionClient('franka_gripper/gripper_action', GripperCommandAction)
    gripper.wait_for_server() 
    open_gripper = GripperCommandGoal()
    open_gripper.command.position = 0.01
    open_gripper.command.max_effort = 0
    gripper.send_goal(open_gripper)
    gripper.wait_for_result()

def close_gripper():
    gripper.wait_for_server() 
    close_gripper = GripperCommandGoal()
    close_gripper.command.position = 0.005
    close_gripper.command.max_effort = 5
    gripper.send_goal(close_gripper)
    gripper.wait_for_result()

def CloseGreifer():
    gripper.wait_for_server() 
    close_gripper = GripperCommandGoal()
    close_gripper.command.position = 0.004
    close_gripper.command.max_effort = 5
    gripper.send_goal(close_gripper)
    gripper.wait_for_result()


def grasp_client(w,s=0.01,f=1,i=0.005,o=0.005):
    # Creates the SimpleActionClient, passing the type of the action
    # (GraspAction) to the constructor.
    global client
    client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)   
    
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal= franka_gripper.msg.GraspGoal()
    goal.width = w
    goal.epsilon.inner = i
    goal.epsilon.outer = o
    goal.speed = s
    goal.force = f
  
    # Sends the goal to the action server.
    client.send_goal(goal)
          
    # Waits for the server to finish performing the action.
    client.wait_for_result()    

    # Prints out the result of executing the action
    return client.get_result() # A GraspResult



def move_client(w,s):
    # Creates the SimpleActionClient, passing the type of the action
    # (MoveAction) to the constructor.

    #client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)    ### grasp --> move
    
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal= franka_gripper.msg.MoveGoal()
    #print(goal)
    goal.width = w
    goal.speed = s
  
    # Sends the goal to the action server.
    client.send_goal(goal)
          
    # Waits for the server to finish performing the action.
    client.wait_for_result()    

    # Prints out the result of executing the action
    return client.get_result() # A GraspResult



        
        
def go_to_joint_state(A):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
    #moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
    #move_group = self.move_group
    #robot = moveit_commander.RobotCommander()
    #move_group = moveit_commander.MoveGroupCommander("panda_arm")
        # We get the joint values from the group and change some of the values:
    move_group.set_planning_pipeline_id('pilz_industrial_motion_planner')
    move_group.set_planner_id('LIN')
    move_group.set_max_velocity_scaling_factor(0.2)
    move_group.set_max_acceleration_scaling_factor(0.08)
    joint_goal = move_group.get_current_joint_values()
  #  joint_goal[0] = 0
  #  joint_goal[1] = -tau / 8
  #  joint_goal[2] = 0
  #  joint_goal[3] = -tau / 4
  #  joint_goal[4] = 0
  #  joint_goal[5] = tau / 6  # 1/6 of a turn
    joint_goal[6] = A #- pi / 2
    move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
        
def GoJoint(A):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
    #moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
    #move_group = self.move_group
    #robot = moveit_commander.RobotCommander()
    #move_group = moveit_commander.MoveGroupCommander("panda_arm")
        # We get the joint values from the group and change some of the values:
    move_group.set_planning_pipeline_id('pilz_industrial_motion_planner')
    move_group.set_planner_id('LIN')
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = A
  #  joint_goal[1] = -tau / 8
  #  joint_goal[2] = 0
  #  joint_goal[3] = -tau / 4
  #  joint_goal[4] = 0
  #  joint_goal[5] = tau / 6  # 1/6 of a turn
  #  joint_goal[6] = A #- pi / 2
    move_group.go(joint_goal, wait=True)
    
        # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
        


def go_home_joints():
    move_group.set_planning_pipeline_id('ompl')
    #move_group.set_planner_id('LIN')
        
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.00018550692740770813
    joint_goal[1] = -0.7856360243361049
    joint_goal[2] = -3.104728356273512e-05
    joint_goal[3] = -2.3559547486104453
    joint_goal[4] = -2.638158408618807e-05
    joint_goal[5] = 1.57174823153471
    joint_goal[6] = 0.785417199086722
    move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)       
        
        
def go_to_position(x,y,z,v=0.1,a=0.008):
        # Copy class variables to local variables to make the web tutorials more clear.
    
    #move_group = moveit_commander.MoveGroupCommander("panda_arm")
    move_group.set_planning_pipeline_id('pilz_industrial_motion_planner')
    move_group.set_planner_id('LIN')
    move_group.set_max_velocity_scaling_factor(v)
    move_group.set_max_acceleration_scaling_factor(a)

        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    HomePose  = move_group.get_current_pose().pose
    pose_goal.orientation= HomePose.orientation # Greifer nicht rotieren
    pose_goal.position.x = x #0.3
    pose_goal.position.y = y #0.3
    pose_goal.position.z = z #0.4

    move_group.set_pose_target(pose_goal)
    success = move_group.go(wait=True)
   
        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
    move_group.clear_pose_targets()
    current_pose = move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


def go_to_pose_goal(x,y,z,v=0.9,a=0.3,W= 0.0004789,Roll= -0.92380,Pitch= 0.382863915, Yaw=0.000316):
        # Copy class variables to local variables to make the web tutorials more clear.
    
    #move_group = moveit_commander.MoveGroupCommander("panda_arm")
    move_group.set_planning_pipeline_id('pilz_industrial_motion_planner')
    move_group.set_planner_id('LIN')
    move_group.set_max_velocity_scaling_factor(v)
    move_group.set_max_acceleration_scaling_factor(a)

        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    HomePose= move_group.get_current_pose().pose
    pose_goal.orientation.w= W
    pose_goal.orientation.x= Roll 
    pose_goal.orientation.y= Pitch 
    pose_goal.orientation.z= Yaw  # Greifer nicht rotieren
    pose_goal.position.x = x #0.3
    pose_goal.position.y = y #0.3
    pose_goal.position.z = z #0.4

    move_group.set_pose_target(pose_goal)
    success = move_group.go(wait=True)
   
        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
    move_group.clear_pose_targets()
    current_pose = move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)
    
    
    
    
def relative_cartesian_path(x=0,y=0,z=0,v=0.01):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
    global move_group 
     
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.x += x
    wpose.position.y += y
    wpose.position.z += z
    waypoints.append(copy.deepcopy(wpose))
    #moveit_msgs.msg.MotionPlanRequest.cartesian_speed_limited_link("panda_hand_tcp")
    moveit_msgs.msg.MotionPlanRequest.max_cartesian_speed=0.001
    move_group.set_planning_time(30)
        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
    
    (plan, fraction) = move_group.compute_cartesian_path(
    	waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold
    #move_group = self.move_group
    #####
    # #TCP=moveit_commander.MoveGroupCommander("panda_hand")
    # move_group.
    # move_group.set_max_cartesian_end_effector_speed(v)
    # move_group.compute_cartesian_path(waypoints)
    # move_group.clear_max_cartesian_end_effector_speed()
    # #####
    move_group.execute(plan, wait=True)

    #return plan, fraction
    
    
def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
    move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL


def addCollisionObjects(planning_scene_interface = moveit_commander.PlanningSceneInterface(ns="/")):
    """Add collision objects to the scene."""
    moveit_commander.roscpp_initialize(sys.argv)

    ## Connect to moveit services ##
    rospy.loginfo(
        "Conneting moveit default moveit 'apply_planning_scene' service.")
    rospy.wait_for_service("apply_planning_scene")
    try:
        planning_scene_srv = rospy.ServiceProxy(
            "apply_planning_scene", ApplyPlanningScene
        )
        rospy.loginfo("Moveit 'apply_planning_scene' service found!")
    except rospy.ServiceException as e:
        rospy.logerr(
            "Moveit 'apply_planning_scene' service initialization failed: %s" % e
        )
        shutdown_msg = "Shutting down %s node because %s service connection failed." % (
            rospy.get_name(),
            planning_scene_srv.resolved_name,
        )
        rospy.logerr(shutdown_msg)
        sys.exit(0)

    ## Create robot commander ##
    robot = moveit_commander.RobotCommander(
        robot_description="robot_description", ns="/"
    )
    rospy.logdebug("Robot Groups: %s", robot.get_group_names())

    ## Create scene commanders ##
    # Used to get information about the world and update the robot
    # its understanding of the world.
    move_group = robot.get_group("panda_arm")
    planning_scene_interface = moveit_commander.PlanningSceneInterface(ns="/")

    ## Specify the planner we want to use ##
    move_group.set_planner_id("TRRTkConfigDefault")

    ## Create a `DisplayTrajectory`_ ROS publisher to display the plan in RVIZ ##
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
    )

    ## Wait a bit for ROS to initialize the planning_scene_interface ##
    rospy.sleep(1.0)

    ## - BEGIN_SUB_TUTORIAL table1 - ##
    # Creating Environment


    # Create vector to hold 3 collision objects. ##
    collision_objects_names = [str for i in range(10)]
    collision_object_sizes = [str for i in range(10)]
    collision_objects = [geometry_msgs.msg.PoseStamped() for i in range(10)]
    # collision_objects_names = [str for i in range(14)]
    # collision_object_sizes = [str for i in range(14)]
    # collision_objects = [geometry_msgs.msg.PoseStamped() for i in range(14)]

    ## Add the first table where the cube will originally be kept. ##
    collision_objects_names[0] = "Box1"
    collision_objects[0].header.frame_id = "panda_link0"

    ## Define the primitive and its dimensions. ##
    collision_object_sizes[0] = (0.36, 0.717, 0.218)  # Box size

    ## Define the pose of the table. ##
    collision_objects[0].pose.position.x = 0.226 +0.28/2 - 0.08 +0.12
    collision_objects[0].pose.position.y = - 0.05 - 0.717/2
    collision_objects[0].pose.position.z = 0 +0.218/2
    ## - END_SUB_TUTORIAL - ##

    ## - BEGIN_SUB_TUTORIAL table2 - ##
    ## Add the second table where we will be placing the cube. ##
    collision_objects_names[1] = "Box2"
    collision_objects[1].header.frame_id = "panda_link0"

    ## Define the primitive and its dimensions. ##
    collision_object_sizes[1] = (0.36, 0.717, 0.218) # Box size

    ## Define the pose of the table. ##
    collision_objects[1].pose.position.x = 0.226 +0.28/2 - 0.08 +0.12
    collision_objects[1].pose.position.y = 0.52 + 0.717/2
    collision_objects[1].pose.position.z = 0  +0.218/2

    ## Define the object that we will be manipulating ##
    collision_objects_names[2] = "Kamera_Stange"
    collision_objects[2].header.frame_id = "panda_link0"

    ## Define the primitive and its dimensions. ##
    collision_object_sizes[2] = (0.08, 0.04, 1.385)  # Box size

    ## Define the pose of the object. ##
    collision_objects[2].pose.position.x = 0.6 + 0.04  - 0.1 +0.12
    collision_objects[2].pose.position.y = 0.46 + 0.02
    collision_objects[2].pose.position.z = 0 + 1.385/2
    ## - END_SUB_TUTORIAL - ##
    ## Define the object that we will be manipulating ##
    collision_objects_names[3] = "Dispenser"
    collision_objects[3].header.frame_id = "panda_link0"

    ## Define the primitive and its dimensions. ##
    collision_object_sizes[3] = (0.240, 0.08, 0.66)  # Box size

    ## Define the pose of the object. ##
    collision_objects[3].pose.position.x = -0.120 + 0.240/2 +0.12
    collision_objects[3].pose.position.y = 0.9 + 0.04
    collision_objects[3].pose.position.z = 0.33
    ## - END_SUB_TUTORIAL - ##
    ## Add the first table where the cube will originally be kept. ##
    collision_objects_names[4] = "Kamera Kopf"
    collision_objects[4].header.frame_id = "panda_link0"

    ## Define the primitive and its dimensions. ##
    collision_object_sizes[4] = (0.35, 0.46, 0.08)  # Box size

    ## Define the pose of the table. ##
    collision_objects[4].pose.position.x = 0.6 + 0.04 - 0.35/2 -0.08 +0.12
    collision_objects[4].pose.position.y = 0.3 + 0.02
    collision_objects[4].pose.position.z = 1.19 - 0.14
    ## - END_SUB_TUTORIAL - ##
    ## Add the first table where the cube will originally be kept. ##
    collision_objects_names[5] = "Dispenser_kopf"
    collision_objects[5].header.frame_id = "panda_link0"

    ## Define the primitive and its dimensions. ##
    collision_object_sizes[5] = (0.24, 0.33, 0.08)  # Box size

    ## Define the pose of the table. ##
    collision_objects[5].pose.position.x = -0.12 + 0.240/2 +0.12
    collision_objects[5].pose.position.y = 0.9 + 0.04 - 0.29
    collision_objects[5].pose.position.z = 0.66 + 0.04
    ## - END_SUB_TUTORIAL - ##

    ## Add the first table where the cube will originally be kept. ##
    collision_objects_names[6] = "Wand"
    collision_objects[6].header.frame_id = "panda_link0"

    ## Define the primitive and its dimensions. ##
    collision_object_sizes[6] = (0.02, 2.0, 1.5)  # Box size

    ## Define the pose of the table. ##
    collision_objects[6].pose.position.x = -0.3 + 0.01 - 0.12
    collision_objects[6].pose.position.y = 0
    collision_objects[6].pose.position.z = 1.5 /2
    ## - END_SUB_TUTORIAL - ##

    ## Add the first table where the cube will originally be kept. ##
    collision_objects_names[7] = "Licht"
    collision_objects[7].header.frame_id = "panda_link0"

    ## Define the primitive and its dimensions. ##
    collision_object_sizes[7] = (0.1, 0.3, 0.3)  # Box size

    ## Define the pose of the table. ##
    collision_objects[7].pose.position.x = 0.6 +0.12
    collision_objects[7].pose.position.y = 0.3
    collision_objects[7].pose.position.z = 0.15
    ## - END_SUB_TUTORIAL - ##

    collision_objects_names[8] = "Vibrationstisch"
    collision_objects[8].header.frame_id = "panda_link0"

    ## Define the primitive and its dimensions. ##
    collision_object_sizes[8] = (0.36, 0.5, 0.218) # Box size

    ## Define the pose of the table. ##
    collision_objects[8].pose.position.x = 0.226 +0.28/2 - 0.08 +0.12
    collision_objects[8].pose.position.y = 0.5/2 + 0.025
    collision_objects[8].pose.position.z = - 0.1 - 0.218/2 

    collision_objects_names[9] = "Vibrationstisch_Wand_Mitte"
    collision_objects[9].header.frame_id = "panda_link0"

    ## Define the primitive and its dimensions. ##
    collision_object_sizes[9] = (0.36, 0.002, 0.1) # Box size

    ## Define the pose of the table. ##
    collision_objects[9].pose.position.x = 0.226 +0.28/2 - 0.08 +0.12
    collision_objects[9].pose.position.y = 0.265
    collision_objects[9].pose.position.z = - 0.1 /2 

    #####################################################################################
    # Wand des Vibrationstisch
    # collision_objects_names[10] = "Vibrationstisch_Wand_1"
    # collision_objects[10].header.frame_id = "panda_link0"

    # ## Define the primitive and its dimensions. ##
    # collision_object_sizes[10] = (0.36, 0.002, 0.1) # Box size

    # ## Define the pose of the table. ##
    # collision_objects[10].pose.position.x = 0.226 +0.28/2 - 0.08 +0.12
    # collision_objects[10].pose.position.y = 0.265 - 0.15
    # collision_objects[10].pose.position.z = - 0.1 /2 

    # collision_objects_names[11] = "Vibrationstisch_Wand_2"
    # collision_objects[11].header.frame_id = "panda_link0"

    # ## Define the primitive and its dimensions. ##
    # collision_object_sizes[11] = (0.36, 0.002, 0.1) # Box size

    # ## Define the pose of the table. ##
    # collision_objects[11].pose.position.x = 0.226 +0.28/2 - 0.08 +0.12
    # collision_objects[11].pose.position.y = 0.265 +0.15
    # collision_objects[11].pose.position.z = - 0.1 /2 

    # collision_objects_names[12] = "Vibrationstisch_Wand_3"
    # collision_objects[12].header.frame_id = "panda_link0"

    # ## Define the primitive and its dimensions. ##
    # collision_object_sizes[12] = (0.002, 0.5, 0.1) # Box size

    # ## Define the pose of the table. ##
    # collision_objects[12].pose.position.x = 0.226     #0.226 +0.28/2 - 0.08 +0.12
    # collision_objects[12].pose.position.y = 0.265
    # collision_objects[12].pose.position.z = - 0.1 /2 

    # collision_objects_names[13] = "Vibrationstisch_Wand_4"
    # collision_objects[13].header.frame_id = "panda_link0"

    # ## Define the primitive and its dimensions. ##
    # collision_object_sizes[13] = (0.002, 0.5, 0.1) # Box size

    # ## Define the pose of the table. ##
    # collision_objects[13].pose.position.x =0.226 +0.375    #0.226 +0.28/2 - 0.08 +0.12
    # collision_objects[13].pose.position.y = 0.265
    # collision_objects[13].pose.position.z = - 0.1 /2 


    ##################################################################################################

    ## Add collision objects to scene ##
    for (name, pose, size) in zip(
            collision_objects_names, collision_objects, collision_object_sizes
    ):
        planning_scene_interface.add_box(name=name, pose=pose, size=size)