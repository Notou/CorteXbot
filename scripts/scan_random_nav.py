#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import rospy
import numpy as np
import os
import time
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import ButtonEvent, WheelDropEvent, Led
from std_srvs.srv import Trigger, TriggerResponse
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
import tf.transformations

maxSpeed = 0.3              # Vitesse maximale
mediumSpeed = 0.2           # Vitesse moyennement limitée
slowSpeed = 0.1             # Vitesse très limitée
emergencyStopDistance = 0.5 # Distance à laquelle le robot s'arrete net
StopDistance = 0.55         # Distance à laquelle le robot freine gentiment pour s'arreter
MediumDistance = 1          # Distance au delà de laquelle le robot va à mediumSpeed
FreedomDistance = 1.5       # Distance au delà de laquelle le robot va à maxSpeed
rotationDistance = 1.5
randomCoefficient = 4
accelerationRate = 0.02
accelerationRateRad = 0.1

minimum_charge = 0.3        # Battery charge to initiate return to base
avoidance_radius = 0.35     # Radius of the robot (m) for obstacle avoidance
epsilon = 1e-20             # To avoid dividing by 0
emergency_stop_dist = 0.4
soft_bounce_dist = 0.5
max_speed_dist = 1.0
max_speed = 1.0
max_rot_speed = 1.5
movements = ["rotation", "translation", "full"]
lidar_rotation_direction = 1 # -1 for clockwise, 1 for counterclockwise

class Walker():
    def __init__(self, ):
        self.pub_safety = rospy.Publisher("/cmd_vel_mux/input/safety_controller",Twist, queue_size=10)
        self.pub_nav = rospy.Publisher("/velocity_smoother/navigator",Twist, queue_size=10)
        self.pub_led1 = rospy.Publisher("/mobile_base/commands/led1", Led, queue_size=10)
        self.pub_led2 = rospy.Publisher("/mobile_base/commands/led2", Led, queue_size=10)
        self.led1 = Led()
        self.led2 = Led()

        self.current_speed = max_speed
        self.current_rot_speed = max_rot_speed
        self.min_draw_dist = 1.0  # 
        self.max_draw_dist = 5.0 # m
        self.max_draw_angle = np.pi/4 # rad


        self.movement_mode = movements[1]
        self.dist_goal = np.inf
        self.angle_goal = 0
        self.last_bounce = 0
        self.rotation_direction = 1 # -1 for clockwise, 1 for counterclockwise

        self.movement_on = False
        self.must_stop = False
        self.random_on = False

        # self.distance_travelled_source = "time"  # "or "odom"
        self.distance_travelled_source = "odom"

        if self.distance_travelled_source == "odom":
            self.current_pose = Pose()

        self.reset_reference()
        # We just rely on yocs_velocity_smoother to manage accelerations


    def scan_callback(self, msg):
        if (not self.movement_on) or self.must_stop:
            return

        if self.movement_mode == "rotation":
            self.rotation_handle()

        if self.movement_mode == "translation":
            rospy.logdebug("Translation")
            self.translation_handle(msg)


    # Detect_obstacles -> return closest collision angle and travel distance to collision
    def check_obstacles_scan(self, msg):
        """
        Check for obstacles in a LaserScan message

        Parameters
        ----------
        msg : LaserScan
            Scan of obstacles

        Returns
        -------
        float
            Angle of closest collision (rad)
        float
            Distance to travel in straight line before collision (m)
        """
        scanned_array = msg.ranges
        scanned_angles = np.arange(msg.angle_min+np.pi, msg.angle_max+np.pi, msg.angle_increment) # Get array of angles corresponding to the measurements
        scanned_angles[len(scanned_angles)//2:] = scanned_angles[len(scanned_angles)//2:] - (2*np.pi)
        scanned_array = np.where(scanned_array<msg.range_min, np.inf, scanned_array)

        # We want to check if there is an obstacle in the way
        collision_distances = (np.sign(np.cos(scanned_angles))*avoidance_radius) / (np.abs(np.sin(scanned_angles)) + epsilon) # Get distances at which we can have a collision in straight line
        travel_to_collision = np.where(scanned_array<collision_distances, scanned_array*np.cos(scanned_angles), np.inf)
        # print(collision_distances[::100])
        # print(travel_to_collision[::100])
        # print(scanned_angles[::100])
        angle_index = np.argmin(travel_to_collision)
        return scanned_angles[angle_index], travel_to_collision[angle_index], angle_index, msg.angle_increment, scanned_array, scanned_angles

    # rotation_handle check if rotation is done, if swich to lin, else, order rotation
    def rotation_handle(self):
        """
        Handle the rotation state of the robot.
        Checks if rotation goal is attaigned and swiches to translation if it is
        """

        if self.movement_mode != "rotation":
            rospy.logerr("Rotation handle called without being in rotation mode!")
            return

        estimated_angle = self.angle_travelled_since_ref(self.reference)
        rospy.loginfo("Estimating an angle of "+str(estimated_angle)+"rad, goal is "+str(self.angle_goal * self.rotation_direction))

        # if estimated_angle * self.rotation_direction >= self.angle_goal:
        if np.abs(estimated_angle) >= np.abs(self.angle_goal):
            # Rotation finished
            self.angle_goal = 0
            self.movement_mode = movements[1]
            self.pub_twist() # Stop movement before netxt callback (to stop rotation)
            self.reset_reference()

            rospy.logdebug("Rotation finished, switching to translation")
        else:
            # Keep rotating
            # if (estimated_angle * self.rotation_direction / self.angle_goal) > 0.9:
            #     desired_rot_speed = 0.5
            # else:
            desired_rot_speed = max_rot_speed
            self.pub_twist(rotation=self.rotation_direction * desired_rot_speed)
        return

    # translation_handle check for obstacle, check for translation done. If then stop and swich to rotation, else continue translation
    def translation_handle(self, msg):
        '''
        Handle the translation state of the robot.
        Check if either translation goal is attaigned or if obstacle is present.
        If it is, change direction and switch to rotation mode.
        '''
        if self.movement_mode != "translation":
            rospy.logerr("Translation handle called without being in translation mode!")
            return

        obstacle_angle, travel_to_obstacle, obstacle_index, angular_rez, distances, angles = self.check_obstacles_scan(msg)
        rospy.loginfo("Closest obstacle reached in "+str(travel_to_obstacle)+"m and is at angle "+str(obstacle_angle))
        if travel_to_obstacle < soft_bounce_dist:
            # We need to stop and then bounce
            self.pub_twist() # Stop movement before next callback (to stop translation)
            self.movement_mode = movements[0]

            # self.angle_goal, self.rotation_direction = self.bounce_angle_dir(obstacle_angle)
            self.angle_goal, self.rotation_direction = self.bounce_angle_dir(obstacle_index, angular_rez, distances, angles)
            self.dist_goal = self.dist_goal - self.distance_travelled_since_ref(self.reference) # Store distance to finish it after rotation
            self.reset_reference()
            rospy.loginfo("Obstacle, bouncing")
        elif self.distance_travelled_since_ref(self.reference) > self.dist_goal:
            # Translation finished
            self.pub_twist() # Stop movement before next callback (to stop translation)
            self.movement_mode = movements[0]

            self.dist_goal, self.angle_goal = self.draw_distance_angle() # Draw new leg of the trip
            self.rotation_direction = np.sign(self.angle_goal)
            self.angle_goal = np.abs(self.angle_goal)
            self.reset_reference()

            rospy.loginfo("Travel leg finished, picking new direction")
        else:
            # Keep translating
            if travel_to_obstacle<max_speed_dist:
                desired_speed = 0.2
            else:
                desired_speed = max_speed
            self.pub_twist(translation=desired_speed)
            rospy.loginfo("Keep translating")
        return

    def reset_reference(self):
        """Reset internal point of reference to current value
        """

        if self.distance_travelled_source == "time":
            self.reference = time.time()

        if self.distance_travelled_source == "odom":
            self.reference = self.current_pose

    # def bounce_angle_dir(self, obstacle_angle_index, angular_resolution, distances, angles):
    #     '''
    #     Compute rotation angle and direction needed to bounce from obstacle
        
    #     Return:
    #     angle (abs value), direction (-1 for clockwise, 1 for counterclockwise)
    #     '''            
    #     angular_aggregation = np.deg2rad(1.0) # Angle over which to aggregate (average) measured distances, to smooth the wall, and absorbant peaks
    #     angular_lookahead = np.deg2rad(5.0) # angle over which to look to estimate orientation of the wall

    #     samples_aggregation = int(angular_aggregation / angular_resolution)
    #     samples_lookahead = int(angular_lookahead / angular_resolution)

    #     mean_dist_obstacle = np.mean(distances[obstacle_angle_index-samples_aggregation:obstacle_angle_index+samples_aggregation])

    #     # Evaluation to the left
    #     mean_dist_lookahead_plus = np.mean(distances[obstacle_angle_index+samples_lookahead-samples_aggregation:obstacle_angle_index+samples_lookahead+samples_aggregation])
    #     delta_sin = np.abs(mean_dist_obstacle*np.sin(angles[obstacle_angle_index]) - mean_dist_lookahead_plus*np.sin(angles[obstacle_angle_index+samples_lookahead]))
    #     delta_cos = np.abs(mean_dist_obstacle*np.cos(angles[obstacle_angle_index]) - mean_dist_lookahead_plus*np.cos(angles[obstacle_angle_index+samples_lookahead]))
    #     desired_angle_plus = np.arctan(delta_sin/delta_cos)

    #     # Evaluation to the right
    #     mean_dist_lookahead_minus = np.mean(distances[obstacle_angle_index-samples_lookahead-samples_aggregation:obstacle_angle_index-samples_lookahead+samples_aggregation])
    #     delta_sin = np.abs(mean_dist_obstacle*np.sin(angles[obstacle_angle_index]) - mean_dist_lookahead_minus*np.sin(angles[obstacle_angle_index+samples_lookahead]))
    #     delta_cos = np.abs(mean_dist_obstacle*np.cos(angles[obstacle_angle_index]) - mean_dist_lookahead_minus*np.cos(angles[obstacle_angle_index+samples_lookahead]))
    #     desired_angle_minus = np.arctan(delta_sin/delta_cos)

    #     angle = np.average([desired_angle_minus, desired_angle_plus]) * 2
    #     direction = -1 * np.sign(np.sin(angles[obstacle_angle_index])) * lidar_rotation_direction
    #     rospy.loginfo("Obstacle at angle "+str(angles[obstacle_angle_index])+" rotating to angle "+str(angle*direction))

    #     if np.isnan(angle):
    #         rospy.loginfo("Estimated angle is NAN, defaulting to small rotation")
    #         angle = 0.5
    #     return angle, direction

    def bounce_angle_dir(self, obstacle_angle_index, angular_resolution, distances, angles):
        '''
        Compute rotation angle and direction needed to bounce from obstacle
        
        Return:
        angle (abs value), direction (-1 for clockwise, 1 for counterclockwise)
        '''        

        # angle = np.clip(0.5 / (time.time()-self.last_bounce), 0.1, 3.0)
        angle = np.clip(0.5 , 0.1, 3.0)
        direction = -1 * np.sign(np.sin(angles[obstacle_angle_index])) * lidar_rotation_direction
        rospy.loginfo("Obstacle at angle "+str(angles[obstacle_angle_index])+" rotating to angle "+str(angle*direction))
        self.last_bounce = time.time()
        return angle, direction

    # def bounce_angle_dir(self, obstacle_angle):
    #     '''
    #     Compute rotation angle and direction needed to bounce from obstacle
        
    #     Return:
    #     angle (abs value), direction (-1 for clockwise, 1 for counterclockwise)
    #     '''            
    #     # WE NEED MORE THAN JUST THE ANGLE!!!
    #     angle = (np.pi/2 - np.abs(obstacle_angle)) * 2#np.maximum(np.pi - (2*np.abs(obstacle_angle)), 0.5)
        
    #     # We want to turn away from the obstacle
    #     direction = -1 * np.sign(np.sin(obstacle_angle)) * lidar_rotation_direction
    #     rospy.loginfo("Obstacle at angle "+str(obstacle_angle)+" rotating to angle "+str(angle*direction))
    #     return angle, direction

    def pub_twist(self, translation=0.0, rotation=0.0):
        '''
        Publish a Twist message on the registered topic with desired translation and rotation speed
        '''
        twist = Twist()
        twist.linear.x = translation
        twist.angular.z = rotation
        self.pub_nav.publish(twist)

    def draw_distance_angle(self):
        '''
        Draw a new distance to travel, and angle to rotate, based on class attributes

        Returns: Distance (m), angle (rad)
        ''' 
        # uniform draw
        distance = np.random.uniform(self.min_draw_dist, self.max_draw_dist)
        angle = np.random.uniform(-self.max_draw_angle, self.max_draw_angle)

        return distance, angle

    def distance_travelled_since_ref(self, reference):
        '''
        Estimate distance travelled since reference point
        '''
        if self.distance_travelled_source == "time":
            time_span = time.time() - reference
            dist = time_span * self.current_speed
        if self.distance_travelled_source == "odom":
            dist = np.sqrt(np.square(self.current_pose.position.x - self.reference.position.x) + \
            np.square(self.current_pose.position.y - self.reference.position.y))
        rospy.loginfo("Estimating a distance of "+str(dist)+"m")
        return dist

    def angle_travelled_since_ref(self, reference):
        '''
        Estimate angle travelled since reference point
        '''

        if self.distance_travelled_source == "time":
            time_span = time.time() - reference
            angle = time_span * self.current_rot_speed * self.rotation_direction
        if self.distance_travelled_source == "odom":
            quat_rotation = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(self.quat_obj_to_array(self.reference.orientation)), self.quat_obj_to_array(self.current_pose.orientation))
            angle = tf.transformations.euler_from_quaternion(quat_rotation)[-1]
        # rospy.loginfo("Estimating an angle of "+str(angle)+"rad")
        return angle

    def quat_obj_to_array(self, obj):
        """
        Pass from Quaternion object from a ros message (geometry_msgs/Quaternion)
        To a numpy array

        Parameters
        ----------
        obj : geometry_msgs/Quaternion

        Returns
        -------
        np.array
        """
        return np.array([obj.x, obj.y, obj.z, obj.w])

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def callback_switch_movement(self, req):
        self.movement_on = not self.movement_on
        self.update_leds()
        rospy.loginfo("Switching movement state to " + str(self.movement_on))
        return TriggerResponse(success=self.movement_on, message="State switched")

    def callback_switch_random(self, req):
        self.random_on = not self.random_on
        if self.random_on:
            self.dist_goal = 0
        else:
            self.dist_goal = np.inf
        self.update_leds()
        rospy.loginfo("Switching random state to " + str(self.random_on))
        return TriggerResponse(success=self.random_on, message="State switched")

    def callback_takeoff(self, req):
        for i in range(10):
            self.pub_twist(translation=-0.3)
            time.sleep(0.1)
        return TriggerResponse(success=True, message="Taking off")

    def callback_goto_base(self, req):        
        self.movement_on = False
        self.update_leds()
        rospy.loginfo("Going back to base, Switching movement state to " + str(self.movement_on))
        self.led1.value = Led.ORANGE
        self.pub_led1.publish(self.led1)
        self.led2.value = Led.ORANGE
        self.pub_led2.publish(self.led2)
        exit_status = os.system("rosrun cortexbot return_home.py")
        if exit_status > 0:
            rospy.logerr("Issue during docking")
            success = False
        else:
            success = True
        return TriggerResponse(success=success, message="")
        
    
    def diagnostics_callback(self, msg): 
        if len(msg.status) == 0:
            return
        for status in msg.status:
            if status.name == "/Power System/Battery":
                for value in status.values:
                    if value.key == 'Percent':
                        charge_percent = float(value.value)
                    if value.key == "Charging State":
                        if value.value == "Trickle Charging" or value.value == "Full Charging":
                            charging = True
                        else:
                            charging = True
        
                if (charge_percent < minimum_charge) and (not charging):
                    rospy.loginfo("Low battery detected, going back to base to recharge")
                    self.callback_goto_base(None)

    def update_leds(self):
        if self.movement_on and self.random_on:
            self.led2.value = Led.GREEN
        elif self.movement_on:            
            self.led2.value = Led.ORANGE
        elif self.random_on:
            self.led2.value = Led.RED
        else:
            self.led2.value = Led.BLACK
        self.pub_led1.publish(self.led1)
        self.pub_led2.publish(self.led2)

    def callback_wheel_drop(self, msg):
        """React to wheels losing contact with ground, or regaining it by triggering movement

        Parameters
        ----------
        msg : WheelDropEvent
            Message recieved from a kobuki topic
        """
        if msg.state > 0:
            # Wheels have dropped
            self.must_stop = True
            self.led1.value = Led.RED
        else:
            self.must_stop = False
            self.led1.value = Led.GREEN
        self.pub_led1.publish(self.led1)
        self.pub_led2.publish(self.led2)

    def callback_buttons(self, msg):
        """
        React to buttons being pressed on the kobuki base
        B0 triggers movement
        B1 triggers random

        Parameters
        ----------
        msg : ButtonEvent
            Button Event
        """
        if msg.state > 0:
            # Ignore button presses, to only consider button releases
            return
        rospy.loginfo("Button click registered, index " + str(msg.button))
        if msg.button == 0:
            self.callback_switch_movement(None)
        if msg.button == 1:
            self.callback_switch_random(None)


    def listener(self):
        rospy.loginfo("lanching nav node")
        rospy.init_node('nav', anonymous=True)
        rospy.Subscriber("/diagnostics", DiagnosticArray, self.diagnostics_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/mobile_base/events/button", ButtonEvent, self.callback_buttons)
        rospy.Subscriber("/mobile_base/events/wheel_drop", WheelDropEvent, self.callback_wheel_drop)
        rospy.Service("/cortexbot/movement_on", Trigger, self.callback_switch_movement)
        rospy.Service("/cortexbot/random_on", Trigger, self.callback_switch_random)
        rospy.Service('/cortexbot/leave_base', Trigger, self.callback_takeoff)
        rospy.Service('/cortexbot/dock', Trigger, self.callback_goto_base)
        if self.distance_travelled_source == "odom":
            rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        self.led1.value = Led.GREEN
        self.pub_led1.publish(self.led1)
        self.led2.value = Led.BLACK
        self.pub_led2.publish(self.led2)
        rospy.spin()

if __name__ == '__main__':
    navigator = Walker()
    navigator.listener()