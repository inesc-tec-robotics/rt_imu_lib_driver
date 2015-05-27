#!/usr/bin/env python

import RTIMU
import rospy
import sys
import os
import rospkg

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import RelativeHumidity
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
from tf import transformations



def imu_driver():
    ##############################################################################################################
    # node initialization
    ##############################################################################################################
    rospy.init_node('rt_imu_lib_driver', anonymous=True)
    rospack = rospkg.RosPack()


    ##############################################################################################################
    # parameters
    ##############################################################################################################
    settings_folder = rospy.get_param('~settings_folder', rospack.get_path('rt_imu_lib_driver') + '/scripts') # .ini configuration folder
    settings_filename = rospy.get_param('~settings_filename', 'RTIMULib') # .ini configuration file name
    msgs_frame_id = rospy.get_param('~msgs_frame_id', 'imu_link')
    msgs_frame_id_ned = rospy.get_param('~msgs_frame_id_ned', 'imu_ned_link') # rosrun tf static_transform_publisher 0 0 0 0.0 0 3.14159265359 imu_ned_link imu_link 10
    orientation_fusion_use_gyro = rospy.get_param('~orientation_fusion_use_gyro', True)
    orientation_fusion_use_accelerometer = rospy.get_param('~orientation_fusion_use_accelerometer', True)
    orientation_fusion_use_compass = rospy.get_param('~orientation_fusion_use_compass', True)
    publish_topics_namespace =rospy.get_param('~publish_topics_namespace', 'imu')
    publish_topic_imu_vector_rpy = rospy.get_param('~publish_topic_imu_vector_rpy', publish_topics_namespace + '/vector_rpy') # if empty no message will be published | enu -> east-north-up
    publish_topic_imu_vector_rpy_ned = rospy.get_param('~publish_topic_imu_vector_rpy_ned', publish_topics_namespace + '/vector_rpy_ned') # if empty no message will be published | ned -> north-east-down
    publish_topic_imu_pose = rospy.get_param('~publish_topic_imu_pose', publish_topics_namespace + '/pose') # if empty no message will be published
    publish_topic_imu_pose_ned = rospy.get_param('~publish_topic_imu_pose_ned', publish_topics_namespace + '/pose_ned') # if empty no message will be published
    publish_topic_imu_pose_with_covariance = rospy.get_param('~publish_topic_imu_pose_with_covariance', publish_topics_namespace + '/pose_with_covariance') # if empty no message will be published
    publish_topic_imu = rospy.get_param('~publish_topic_imu', publish_topics_namespace + '/imu') # if empty no message will be published
    publish_topic_magnetic_field = rospy.get_param('~publish_topic_magnetic_field', publish_topics_namespace + '/magnetic_field') # if empty no message will be published
    publish_topic_pressure = rospy.get_param('~publish_topic_pressure', publish_topics_namespace + '/pressure') # if empty no message will be published
    publish_topic_temperature = rospy.get_param('~publish_topic_temperature', publish_topics_namespace + '/temperature') # if empty no message will be published
    publish_topic_humidity = rospy.get_param('~publish_topic_humidity', publish_topics_namespace + '/humidity') # if empty no message will be published
    imu_orientation_covariance = rospy.get_param('~imu_orientation_covariance', 0.0005)
    imu_angular_velocity_covariance = rospy.get_param('~imu_angular_velocity_covariance', 0.001)
    imu_linear_acceleration_covariance = rospy.get_param('~imu_linear_acceleration_covariance', 0.0005)
    magnetic_field_covariance = rospy.get_param('~magnetic_field_covariance', 0.01)
    pressure_variance = rospy.get_param('~pressure_variance', 0.0)
    temperature_variance = rospy.get_param('~temperature_variance', 0.0)
    humidity_variance = rospy.get_param('~humidity_variance', 0.0)
    gravity_acceleration = rospy.get_param('~gravity_acceleration', 9.80665)
    msg_header_use_trimulib_time = rospy.get_param('~msg_header_use_trimulib_time', True) # False uses rospy.Time.now()


    ##############################################################################################################
    # published topics
    ##############################################################################################################
    publisher_vector_rpy = rospy.Publisher(publish_topic_imu_vector_rpy, Vector3Stamped, queue_size=10)
    publisher_vector_rpy_ned = rospy.Publisher(publish_topic_imu_vector_rpy_ned, Vector3Stamped, queue_size=10)
    publisher_pose = rospy.Publisher(publish_topic_imu_pose, PoseStamped, queue_size=10)
    publisher_pose_ned = rospy.Publisher(publish_topic_imu_pose_ned, PoseStamped, queue_size=10)
    publisher_pose_with_covariance = rospy.Publisher(publish_topic_imu_pose_with_covariance, PoseWithCovarianceStamped, queue_size=10)
    publisher_imu = rospy.Publisher(publish_topic_imu, Imu, queue_size=10)
    publisher_magnetic_field = rospy.Publisher(publish_topic_magnetic_field, MagneticField, queue_size=10)
    publisher_pressure = rospy.Publisher(publish_topic_pressure, FluidPressure, queue_size=10)
    publisher_temperature = rospy.Publisher(publish_topic_temperature, Temperature, queue_size=10)
    publisher_humidity = rospy.Publisher(publish_topic_humidity, RelativeHumidity, queue_size=10)


    ##############################################################################################################
    # driver initialization
    ##############################################################################################################
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

    imu_driver.setGyroEnable(orientation_fusion_use_gyro)
    imu_driver.setAccelEnable(orientation_fusion_use_accelerometer)
    imu_driver.setCompassEnable(orientation_fusion_use_compass)

    poll_interval = imu_driver.IMUGetPollInterval()
    rate = 1000 / poll_interval
    rospy.loginfo("IMU polling interval: %s ms | rate: %s hz", poll_interval, rate)


    ##############################################################################################################
    # constant message fields setup
    ##############################################################################################################
    if publish_topic_imu_vector_rpy:
        vector_rpy_msg = Vector3Stamped()
        vector_rpy_msg.header.frame_id = msgs_frame_id
    
    if publish_topic_imu_vector_rpy_ned:
        vector_rpy_ned_msg = Vector3Stamped()
        vector_rpy_ned_msg.header.frame_id = msgs_frame_id_ned
    
    if publish_topic_imu_pose:
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = msgs_frame_id
    
    if publish_topic_imu_pose_ned:
        pose_ned_msg = PoseStamped()
        pose_ned_msg.header.frame_id = msgs_frame_id_ned
    
    if publish_topic_imu_pose_with_covariance:
        pose_with_covariance_msg = PoseWithCovarianceStamped()
        pose_with_covariance_msg.header.frame_id = msgs_frame_id
        pose_with_covariance_msg.pose.covariance[0] = 1e6 # ignore position
        pose_with_covariance_msg.pose.covariance[7] = 1e6 # ignore position
        pose_with_covariance_msg.pose.covariance[14] = 1e6 # ignore position
        pose_with_covariance_msg.pose.covariance[21] = imu_orientation_covariance
        pose_with_covariance_msg.pose.covariance[28] = imu_orientation_covariance
        pose_with_covariance_msg.pose.covariance[35] = imu_orientation_covariance

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

    if publish_topic_pressure:
        pressure_msg = FluidPressure()
        pressure_msg.header.frame_id = msgs_frame_id
        pressure_msg.variance = pressure_variance
    
    if publish_topic_temperature:
        temperature_msg = Temperature()
        temperature_msg.header.frame_id = msgs_frame_id
        temperature_msg.variance = temperature_variance
        
    if publish_topic_humidity:
        humidity_msg = RelativeHumidity()
        humidity_msg.header.frame_id = msgs_frame_id
        humidity_msg.variance = humidity_variance


    ##############################################################################################################
    # imu msg publishing
    ##############################################################################################################
    polling_rate = rospy.Rate(rate)
    while not rospy.is_shutdown():
        if imu_driver.IMURead():
            current_time = rospy.Time.now()
            imu_data = imu_driver.getIMUData()
            rospy.logdebug("Time stamp -> Driver: %f | ROS: %f", imu_data["timestamp"] / 1e6, current_time.to_sec())
            time_difference_rtimulib_vs_ros = abs(imu_data["timestamp"] / 1e6 - current_time.to_sec())
            if time_difference_rtimulib_vs_ros > 0.2:
                rospy.logwarn("High time difference (%s sec) between RTIMULib and rospy.Time.now()", time_difference_rtimulib_vs_ros)


            ##############################################################################################################
            # msg headers time
            ##############################################################################################################
            if msg_header_use_trimulib_time:
                trimulib_time = rospy.Time(imu_data["timestamp"] / 1e6)
                if publish_topic_imu_vector_rpy:
                    vector_rpy_msg.header.stamp = trimulib_time
                if publish_topic_imu_vector_rpy_ned:
                    vector_rpy_ned_msg.header.stamp = trimulib_time
                if publish_topic_imu_pose:
                    pose_msg.header.stamp = trimulib_time
                if publish_topic_imu_pose_ned:
                    pose_ned_msg.header.stamp = trimulib_time
                if publish_topic_imu_pose_with_covariance:
                    pose_with_covariance_msg.header.stamp = trimulib_time
                if publish_topic_imu:
                    imu_msg.header.stamp = trimulib_time
                if publish_topic_magnetic_field:
                    magnetic_field_msg.header.stamp = trimulib_time
                if publish_topic_pressure:
                    pressure_msg.header.stamp = trimulib_time
                if publish_topic_temperature:
                    temperature_msg.header.stamp = trimulib_time
                if publish_topic_humidity:
                    humidity_msg.header.stamp = trimulib_time
            else:
                if publish_topic_imu_vector_rpy:
                    vector_rpy_msg.header.stamp = current_time
                if publish_topic_imu_vector_rpy_ned:
                    vector_rpy_ned_msg.header.stamp = current_time
                if publish_topic_imu_pose:
                    pose_msg.header.stamp = current_time
                if publish_topic_imu_pose_ned:
                    pose_ned_msg.header.stamp = current_time
                if publish_topic_imu_pose_with_covariance:
                    pose_with_covariance_msg.header.stamp = current_time
                if publish_topic_imu:
                    imu_msg.header.stamp = current_time
                if publish_topic_magnetic_field:
                    magnetic_field_msg.header.stamp = current_time
                if publish_topic_pressure:
                    pressure_msg.header.stamp = current_time
                if publish_topic_temperature:
                    temperature_msg.header.stamp = current_time
                if publish_topic_humidity:
                    humidity_msg.header.stamp = current_time


            ##############################################################################################################
            # Conversion between NED (North-East-Down) to ROS coordinate frame Front-Left-Up
            ##############################################################################################################
            if imu_data["fusionQPoseValid"]:
                imu_quaternion_qw_ros = imu_data["fusionQPose"][0]
                imu_quaternion_qx_ros = imu_data["fusionQPose"][1]
                imu_quaternion_qy_ros = -imu_data["fusionQPose"][2]
                imu_quaternion_qz_ros = -imu_data["fusionQPose"][3]


            ##############################################################################################################
            # vector_rpy msg
            ##############################################################################################################
            if publish_topic_imu_vector_rpy:
                if imu_data["fusionQPoseValid"]:
                    imu_rotation_ros = transformations.quaternion_matrix([imu_quaternion_qx_ros, imu_quaternion_qy_ros, imu_quaternion_qz_ros, imu_quaternion_qw_ros])
                    yaw, pitch, roll = transformations.euler_from_matrix(imu_rotation_ros, 'rzyx')
                    vector_rpy_msg.vector.x = roll
                    vector_rpy_msg.vector.y = pitch
                    vector_rpy_msg.vector.z = yaw
                    publisher_vector_rpy.publish(vector_rpy_msg)

            if publish_topic_imu_vector_rpy_ned:
                if imu_data["fusionPoseValid"]:
                    vector_rpy_ned_msg.vector.x = imu_data["fusionPose"][0]
                    vector_rpy_ned_msg.vector.y = imu_data["fusionPose"][1]
                    vector_rpy_ned_msg.vector.z = imu_data["fusionPose"][2]
                    publisher_vector_rpy_ned.publish(vector_rpy_ned_msg)


            ##############################################################################################################
            # pose msg
            ##############################################################################################################
            if publish_topic_imu_pose:
                if imu_data["fusionQPoseValid"]:
                    pose_msg.pose.orientation.w = imu_quaternion_qw_ros
                    pose_msg.pose.orientation.x = imu_quaternion_qx_ros
                    pose_msg.pose.orientation.y = imu_quaternion_qy_ros
                    pose_msg.pose.orientation.z = imu_quaternion_qz_ros
                    publisher_pose.publish(pose_msg)

            if publish_topic_imu_pose_ned:
                if imu_data["fusionQPoseValid"]:
                    pose_ned_msg.pose.orientation.w = imu_data["fusionQPose"][0]
                    pose_ned_msg.pose.orientation.x = imu_data["fusionQPose"][1]
                    pose_ned_msg.pose.orientation.y = imu_data["fusionQPose"][2]
                    pose_ned_msg.pose.orientation.z = imu_data["fusionQPose"][3]
                    publisher_pose_ned.publish(pose_ned_msg)

            ##############################################################################################################
            # pose with covariance msg
            ##############################################################################################################
            if publish_topic_imu_pose_with_covariance:
                if imu_data["fusionQPoseValid"]:
                    pose_with_covariance_msg.pose.pose.orientation.w = imu_quaternion_qw_ros
                    pose_with_covariance_msg.pose.pose.orientation.x = imu_quaternion_qx_ros
                    pose_with_covariance_msg.pose.pose.orientation.y = imu_quaternion_qy_ros
                    pose_with_covariance_msg.pose.pose.orientation.z = imu_quaternion_qz_ros
                    publisher_pose_with_covariance.publish(pose_with_covariance_msg)


            ##############################################################################################################
            # imu msg
            ##############################################################################################################
            if publish_topic_imu:
                if imu_data["fusionQPoseValid"] and imu_data["gyroValid"] and imu_data["accelValid"]:
                    imu_msg.orientation.w = imu_quaternion_qw_ros
                    imu_msg.orientation.x = imu_quaternion_qx_ros
                    imu_msg.orientation.y = imu_quaternion_qy_ros
                    imu_msg.orientation.z = imu_quaternion_qz_ros
                    # gyro data in radians/sec
                    imu_msg.angular_velocity.x = imu_data["gyro"][0]
                    imu_msg.angular_velocity.y = -imu_data["gyro"][1]
                    imu_msg.angular_velocity.z = -imu_data["gyro"][2]
                    # accel data in gs
                    imu_msg.linear_acceleration.x = -imu_data["accel"][0] * gravity_acceleration # in m/s^2
                    imu_msg.linear_acceleration.y = imu_data["accel"][1] * gravity_acceleration # in m/s^2
                    imu_msg.linear_acceleration.z = imu_data["accel"][2] * gravity_acceleration # in m/s^2
                    publisher_imu.publish(imu_msg)


            ##############################################################################################################
            # magnetic field msg
            ##############################################################################################################
            if publish_topic_magnetic_field:
                if imu_data["compassValid"]:
                    # compass data in uT
                    magnetic_field_msg.magnetic_field.x = imu_data["compass"][0] / 1e6 # in Tesla
                    magnetic_field_msg.magnetic_field.y = imu_data["compass"][1] / 1e6 # in Tesla
                    magnetic_field_msg.magnetic_field.z = imu_data["compass"][2] / 1e6 # in Tesla
                    publisher_magnetic_field.publish(magnetic_field_msg)


            ##############################################################################################################
            # pressure field msg
            ##############################################################################################################
            if publish_topic_pressure:
                if imu_data["pressureValid"]:
                    pressure_msg.fluid_pressure = imu_data["pressure"] # absolute pressure reading in Pascals
                    publisher_pressure.publish(pressure_msg)


            ##############################################################################################################
            # temperature field msg
            ##############################################################################################################
            if publish_topic_temperature:
                if imu_data["temperatureValid"]:
                    temperature_msg.temperature = imu_data["temperature"] # temperature in Degrees Celsius
                    publisher_temperature.publish(temperature_msg)


            ##############################################################################################################
            # magnetic field msg
            ##############################################################################################################
            if publish_topic_humidity:
                if imu_data["humidityValid"]:
                    humidity_msg.relative_humidity = imu_data["humidity"] # ratio of partial pressure of water vapor to the saturated vapor pressure at a temperature
                    publisher_humidity.publish(humidity_msg)

        polling_rate.sleep()



if __name__ == '__main__':
    try:
        imu_driver()
    except rospy.ROSInterruptException:
        rospy.logfatal("IMU driver failed to initialize")
        sys.exit(2)

