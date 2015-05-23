#!/usr/bin/env python

import RTIMU
import rospy
import sys
import os

from tf import transformations
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import PoseWithCovarianceStamped


def imu_driver():
    # node initialization
    rospy.init_node('rt_imu_lib_driver', anonymous=True)


    # parameters
    settings_folder = rospy.get_param('~settings_folder', os.getcwd()) # .ini configuration folder
    settings_filename = rospy.get_param('~settings_filename', 'RTIMULib') # .ini configuration file
    msgs_frame_id = rospy.get_param('~msgs_frame_id', 'imu_link')
    publish_topic_imu_pose = rospy.get_param('~publish_topic_imu_pose', 'imu_pose') # if empty no message will be published
    publish_topic_imu = rospy.get_param('~publish_topic_imu', 'imu') # if empty no message will be published
    publish_topic_magnetic_field = rospy.get_param('~publish_topic_magnetic_field', "magnetic_field") # if empty no message will be published
    imu_orientation_covariance = rospy.get_param('~imu_orientation_covariance', 0.0005)
    imu_angular_velocity_covariance = rospy.get_param('~imu_angular_velocity_covariance', 0.001)
    imu_linear_acceleration_covariance = rospy.get_param('~imu_linear_acceleration_covariance', 0.0005)
    magnetic_field_covariance = rospy.get_param('~magnetic_field_covariance', 0.01)
    gravity_acceleration = rospy.get_param('~gravity_acceleration', 9.80665)
    msg_header_use_trimulib_time = rospy.get_param('~msg_header_use_trimulib_time', True) # False uses rospy.Time.now()


    # published topics
    publisher_pose = rospy.Publisher(publish_topic_imu_pose, PoseWithCovarianceStamped, queue_size=10)
    publisher_imu = rospy.Publisher(publish_topic_imu, Imu, queue_size=10)
    publisher_magnetic_field = rospy.Publisher(publish_topic_magnetic_field, MagneticField, queue_size=10)


    # driver initialization
    driver_settings_file = settings_folder + '/' + settings_filename
    if os.path.exists(driver_settings_file + ".ini"):
        rospy.loginfo("Using IMU configuration file %s.ini", driver_settings_file)
    else:
        rospy.logwarn("Missing %s.ini configuration file (creating one with default configurations...)", driver_settings_file)
      
      
    driver_settings = RTIMU.Settings(driver_settings_file)
    imu_driver = RTIMU.RTIMU(driver_settings)

    rospy.loginfo("IMU model: %s", imu_driver.IMUName())
    if (not imu_driver.IMUInit()):
        rospy.logerr("IMU driver library failed to initialize")
        sys.exit(1)
    else:
        rospy.logdebug("IMU initialization successful")

    poll_interval = imu_driver.IMUGetPollInterval()
    rate = 1000 / poll_interval
    rospy.loginfo("IMU polling interval: %s ms | rate: %s hz", poll_interval, rate)


    # constant message fields setup
    if publish_topic_imu_pose:
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = msgs_frame_id
        pose_msg.pose.covariance[0] = 1e6 # ignore position
        pose_msg.pose.covariance[7] = 1e6 # ignore position
        pose_msg.pose.covariance[14] = 1e6 # ignore position
        pose_msg.pose.covariance[21] = imu_orientation_covariance
        pose_msg.pose.covariance[28] = imu_orientation_covariance
        pose_msg.pose.covariance[35] = imu_orientation_covariance

    if publish_topic_imu:
        imu_msg = Imu()
        imu_msg.header.frame_id = msgs_frame_id
        imu_msg.orientation_covariance[0] = imu_orientation_covariance
        imu_msg.orientation_covariance[4] = imu_orientation_covariance
        imu_msg.orientation_covariance[8] = imu_orientation_covariance
        imu_msg.angular_velocity_covariance[0] = imu_angular_velocity_covariance
        imu_msg.angular_velocity_covariance[4] = imu_angular_velocity_covariance
        imu_msg.angular_velocity_covariance[8] = imu_angular_velocity_covariance
        imu_msg.linear_acceleration_covariance[0] = imu_linear_acceleration_covariance
        imu_msg.linear_acceleration_covariance[4] = imu_linear_acceleration_covariance
        imu_msg.linear_acceleration_covariance[8] = imu_linear_acceleration_covariance

    if publish_topic_magnetic_field:
        magnetic_field_msg = MagneticField()
        magnetic_field_msg.header.frame_id = msgs_frame_id
        magnetic_field_msg.magnetic_field_covariance[0] = magnetic_field_covariance
        magnetic_field_msg.magnetic_field_covariance[4] = magnetic_field_covariance
        magnetic_field_msg.magnetic_field_covariance[8] = magnetic_field_covariance

    # imu msg publishing
    polling_rate = rospy.Rate(rate)
    while not rospy.is_shutdown():
        if imu_driver.IMURead():
            current_time = rospy.Time.now()
            imu_data = imu_driver.getIMUData()
            rospy.logdebug("Time stamp -> Driver: %f | ROS: %f", imu_data["timestamp"] / 1e6, current_time.to_sec())
            
            # msg headers time
            if msg_header_use_trimulib_time:
                if publish_topic_imu_pose:
                    pose_msg.header.stamp = rospy.Time(imu_data["timestamp"] / 1e6)
                if publish_topic_imu:
                    imu_msg.header.stamp = rospy.Time(imu_data["timestamp"] / 1e6)
                if publish_topic_magnetic_field:
                    magnetic_field_msg.header.stamp = rospy.Time(imu_data["timestamp"] / 1e6)
            else:
                if publish_topic_imu_pose:
                    pose_msg.header.stamp = current_time
                if publish_topic_imu:
                    imu_msg.header.stamp = current_time
                if publish_topic_magnetic_field:
                    magnetic_field_msg.header.stamp = current_time


            ##############################################################################################################
            # pose msg
            ##############################################################################################################
            if publish_topic_imu_pose:
                pose_msg.pose.pose.orientation.w = imu_data["fusionQPose"][0]
                pose_msg.pose.pose.orientation.x = imu_data["fusionQPose"][1]
                pose_msg.pose.pose.orientation.y = imu_data["fusionQPose"][2]
                pose_msg.pose.pose.orientation.z = imu_data["fusionQPose"][3]
                publisher_pose.publish(pose_msg)


            ##############################################################################################################
            # imu msg
            ##############################################################################################################
            if publish_topic_imu:
                imu_msg.orientation.w = imu_data["fusionQPose"][0]
                imu_msg.orientation.x = imu_data["fusionQPose"][1]
                imu_msg.orientation.y = imu_data["fusionQPose"][2]
                imu_msg.orientation.z = imu_data["fusionQPose"][3]
                # gyro rates in radians/sec
                imu_msg.angular_velocity.x = imu_data["gyro"][0]
                imu_msg.angular_velocity.y = imu_data["gyro"][1]
                imu_msg.angular_velocity.z = imu_data["gyro"][2]
                # accel data in gs
                imu_msg.linear_acceleration.x = imu_data["accel"][0] * gravity_acceleration
                imu_msg.linear_acceleration.y = imu_data["accel"][1] * gravity_acceleration
                imu_msg.linear_acceleration.z = imu_data["accel"][2] * gravity_acceleration
                publisher_imu.publish(imu_msg)


            ##############################################################################################################
            # magnetic field msg
            ##############################################################################################################
            # compass data in uT
            if publish_topic_magnetic_field:
                magnetic_field_msg.magnetic_field.x = imu_data["compass"][0] * 1e6
                magnetic_field_msg.magnetic_field.y = imu_data["compass"][1] * 1e6
                magnetic_field_msg.magnetic_field.z = imu_data["compass"][2] * 1e6
                publisher_magnetic_field.publish(magnetic_field_msg)

        polling_rate.sleep()



if __name__ == '__main__':
    try:
        imu_driver()
    except rospy.ROSInterruptException:
        rospy.logfatal("IMU driver failed to initialize")
        sys.exit(2)

