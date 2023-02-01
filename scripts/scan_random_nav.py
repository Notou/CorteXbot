import rospy
import numpy as np
import math
import time
from geometry_msgs.msg import Twist
from sensor_msgs import LaserScan
from kobuki_msgs import ButtonEvent, WheelDropEvent
from std_msgs.srv import String, Bool
from std_srvs.msg import Empty, Trigger

maxSpeed = 0.3              # Vitesse maximale
mediumSpeed = 0.2           # Vitesse moyennement limitée
slowSpeed = 0.1             # Vitesse très limitée
emergencyStopDistance = 0.5 # Distance à laquelle le robot s'arrete net
StopDistance = 0.55         # Distance à laquelle le robot freine gentiment pour s'arreter
MediumDistance = 1          # Distance au delà de laquelle le robo va à mediumSpeed
FreedomDistance = 1.5       # Distance au delà de laquelle le robot va à maxSpeed
rotationDistance = 1.5
randomCoefficient = 4
accelerationRate = 0.02
accelerationRateRad = 0.1

avoidance_radius = 0.3      # Radius of the robot (m) for obstacle avoidance
epsilon = 1e-20             # To avoid dividing by 0
emergency_stop_dist = 0.4
soft_bounce_dist = 0.5
max_speed = 0.3
max_rot_speed = 3.0
movements = ["rotation", "translation", "full"]
lidar_rotation_direction = -1 # -1 for clockwise, 1 for counterclockwise

class Walker():
    def __init__(self, ):
        self.pub_safety = rospy.self.publisher("/cmd_vel_mux/input/safety_controller",Twist, queue_size=10)
        self.pub_nav = rospy.self.publisher("/velocity_smoother/nav",Twist, queue_size=10)

        self.current_speed = 0
        self.current_rot_speed = 0
        self.min_draw_dist = 2  # m
        self.max_draw_dist = 10 # m
        self.max_draw_angle = np.pi # rad


        self.movement_mode = movements[1]
        self.dist_goal = 0
        self.angle_goal = 0
        self.rotation_direction = 1 # -1 for clockwise, 1 for counterclockwise
        self.reset_reference()

        self.movement_on = False
        self.must_stop = False
        self.random_on = False

        self.distance_travelled_source = "time"  # "or "odom"
        # We just rely on yocs_velocity_smoother to manage accelerations


    def scan_callback(self, msg):
        if (not self.movement_on) or self.must_stop:
            return

        if self.movement_mode == "rotation":
            self.rotation_handle()

        if self.movement_mode == "translation":
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
        scanned_angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increments) # Get array of angles corresponding to the measurements

        forward_angles = scanned_angles[np.cos(scanned_angles)>0] # Only keep what's forwards
        forward_distances = scanned_array[np.cos(scanned_angles)>0]

        # We want to check if there is an obstacle in the way
        collision_distances = avoidance_radius / (np.sin(forward_angles) + epsilon) # Get distances at which we can have a collision in straight line
        travel_to_collision = np.where(forward_distances<collision_distances, forward_distances*np.cos(forward_angles), np.inf)

        angle_index = np.argmin(travel_to_collision)
        return forward_angles[angle_index], travel_to_collision[angle_index]

    # rotation_handle check if rotation is done, if swich to lin, else, order rotation
    def rotation_handle(self):
        """
        Handle the rotation state of the robot.
        Checks if rotation goal is attaigned and swiches to translation if it is
        """

        if self.movement_mode != "rotation":
            rospy.logerr("Rotation handle called without being in rotation mode!")
            return
        
        if self.angle_travelled_since_ref(self.reference) >= self.angle_goal:
            # Rotation finished
            self.angle_goal = 0
            self.movement_mode = movements[1]
            self.pub_twist() # Stop movement before netxt callback (to stop rotation)
            self.reset_reference()

            rospy.logdebug("Rotation finished, switching to translation")
        else:
            # Keep rotating
            self.pub_twist(rotation=self.rotation_direction * max_rot_speed)
        return

    # translation_handle check for obstacle, check for translation done. If then stop and swich to rotation, else continue translation
    def translation_handle(self, msg: LaserScan):
        '''
        Handle the translation state of the robot.
        Check if either translation goal is attaigned or if obstacle is present.
        If it is, change direction and switch to rotation mode.
        '''
        if self.movement_mode != "translation":
            rospy.logerr("Translation handle called without being in translation mode!")
            return

        travel_to_obstacle, obstacle_angle = self.check_obstacles_scan(msg)
        if travel_to_obstacle < soft_bounce_dist:
            # We need to stop and then bounce
            self.pub_twist() # Stop movement before netxt callback (to stop translation)
            self.movement_mode = movements[0]

            self.angle_goal, self.rotation_direction = self.bounce_angle_dir(obstacle_angle)
            self.dist_goal = self.dist_goal - self.distance_travelled_since_ref(self.reference) # Store distance to finish it after rotation
            self.reset_reference()
            rospy.logdebug("Obstacle, bouncing")
        elif self.distance_travelled_since_ref(self.reference) > self.dist_goal:
            # Translation finished
            self.pub_twist() # Stop movement before next callback (to stop translation)
            self.movement_mode = movements[0]

            self.dist_goal, self.angle_goal = self.draw_distance_angle() # Draw new leg of the trip
            self.reset_reference()

            rospy.logdebug("Travel leg finished, picking new direction")
        else:
            # Keep translating
            self.pub_twist(translation=max_speed)
        return

    def reset_reference(self):
        """Reset internal point of reference to current value
        """

        if self.distance_travelled_source == "time":
            self.reference = time.time()

    def bounce_angle_dir(self, obstacle_angle: float):
        '''
        Compute rotation angle and direction needed to bounce from obstacle
        
        Return:
        angle (abs value), direction (-1 for clockwise, 1 for counterclockwise)
        '''            
        angle = np.pi - (2*np.abs(obstacle_angle))
        
        # We want to turn away from the obstacle
        direction = -1 * np.sign(obstacle_angle) * lidar_rotation_direction

        return angle, direction

    def pub_twist(self, translation: float=0, rotation: float=0):
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
            dist = time_span / self.current_speed
        return dist

    def angle_travelled_since_ref(self, reference):
        '''
        Estimate angle travelled since reference point
        '''

        if self.distance_travelled_source == "time":
            time_span = time.time() - reference
            angle = time_span / self.current_rot_speed
        return angle

    def callback_switch_movement(self, req):
        self.movement_on != self.movement_on

    def callback_switch_random(self, req):
        self.dist_goal = np.where(self.dist_goal < np.inf, np.inf, 0)

    def callback_wheel_drop(self, msg: WheelDropEvent):
        """React to wheels losing contact with ground, or regaining it by triggering movement

        Parameters
        ----------
        msg : WheelDropEvent
            Message recieved from a kobuki topic
        """
        if msg.state > 0:
            # Wheels have dropped
            self.must_stop = True
        else:
            self.must_stop = False

    def callback_buttons(self, msg: ButtonEvent):
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

        if msg.button == 0:
            self.callback_switch_movement(None)
        if msg.button == 1:
            self.callback_switch_random(None)


    def listener(self):
        rospy.loginfo("lanching nav node")
        rospy.init_node('nav', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/mobile_base/events/button", ButtonEvent, self.callback_buttons)
        rospy.Subscriber("/mobile_base/events/wheel_drop", WheelDropEvent, self.callback_buttons)
        rospy.Service("/cortexbot/movement_on", Trigger, self.callback_switch_movement)
        rospy.Service("/cortexbot/random_on", Trigger, self.callback_switch_random)
        # rospy.Service('/cortexbot/leave_base', Trigger, self.callbackSwitchTakeoff)
        # rospy.Service('/cortexbot/dock', Trigger, self.GotToBaseCallback)
        rospy.spin()

if __name__ == '__main__':
    navigator = Walker()
    navigator.listener()