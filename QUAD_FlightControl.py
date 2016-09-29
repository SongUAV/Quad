#-------------------------------------------------------------------------------
# Name:        Flight control
# Purpose:      control the drone do the following action:
#               1) landing on the platform, 2) stabilizing in the air
#               3) yawing to a desired angle
#
# Author:      Song
#
# Created:     27/09/2016
# Copyright:   (c) Reforges 2016
# Licence:     <your licence>
#-------------------------------------------------------------------------------
# PID parameters
IMU_PITCH_ROLL_Kp = 0.25
IMU_PITCH_ROLL_Kd = 2.1
IMU_PITCH_ROLL_Ki = 0.001

IMU_YAW_Kp = 3.0
IMU_YAW_Kd = 5.0
IMU_YAW_Ki = 0.0

ALTITUDE_Kp = 40.0
ALTITUDE_Ki = 0.1
ALTITUDE_Kd = 1550

GPS_X_Y_Kp = 3.0
GPS_X_Y_Ki = 0.0
GPS_X_Y_Kd = 5.0
# position of the platform

from pidcontrol import Stability_PID_Controller, Yaw_PID_Controller, Hover_PID_Controller, GPS_PID_Controller
import math
import PathOpt
class FlightControl():
     def __init__(self, logfile=None):
        '''
        Creates a new Quadrotor object with optional logfile.
        '''

        # Store logfile handle
        self.logfile = logfile

        # Create PD controllers for pitch, roll based on angles from Inertial Measurement Unit (IMU)
        self.pitch_Stability_PID = Stability_PID_Controller(IMU_PITCH_ROLL_Kp, IMU_PITCH_ROLL_Kd, IMU_PITCH_ROLL_Ki)
        self.roll_Stability_PID  = Stability_PID_Controller(IMU_PITCH_ROLL_Kp, IMU_PITCH_ROLL_Kd, IMU_PITCH_ROLL_Ki)

        # Special handling for yaw from IMU
        self.yaw_IMU_PID   = Yaw_PID_Controller(IMU_YAW_Kp, IMU_YAW_Kd, IMU_YAW_Ki)

        # PID controller for position
        self.x_PID= GPS_PID_Controller(GPS_X_Y_Kp, GPS_X_Y_Kd, GPS_X_Y_Ki)
        self.y_PID= GPS_PID_Controller(GPS_X_Y_Kp, GPS_X_Y_Kd, GPS_X_Y_Ki)

        # Create PID controller for altitude-hold
        self.altitude_PID = Hover_PID_Controller(ALTITUDE_Kp, ALTITUDE_Kd, ALTITUDE_Ki)

        # for path optimization
        self.T_p = 5
        self.time_step = 0.01

        # platform
        self.X_platform = 0.0
        self.Y_platform = 0.0
        self.Z_platform = 0.0

        # For landing
        self.Init_GPS_X = 0.0
        self.Init_GPS_Y = 0.0
        self.Init_Altitude = 0.0

        # For yawing
        self.Init_yaw = 0.0

        # land the drone on the platform
     def Landing(self, roll, pitch, yaw, X_gps, Y_gps, Z_gps):
        # 1) roll, pitch and yaw should be roughly zero
        # 2) keep the horizontal position
        # 3) lower the altitude (the overshoot should be as small as possible)

        # optimize the altitude path
        self.altitude_PID.t += 1
        target_altitude_timestep = PathOpt(self.T_p, self.Init_Altitude, self.Z_platform, self.altitude_PID.t, self.time_step)

        # Get PID altitude correction if indicated
        altitudeCorrection = self.altitude_PID.getCorrection(Z_gps, target_altitude_timestep, self.time_step)

        # PID control for pitch, roll based on angles from Inertial Measurement Unit (IMU)
        imuPitchCorrection = self.pitch_Stability_PID.getCorrection(pitch, self.time_step)
        imuRollCorrection  = self.roll_Stability_PID.getCorrection(-roll, self.time_step)

        # PID control for horizontal position

        PositionXCorrection = self.x_PID.getCorrection(self.X_platform, X_gps, self.time_step)
        PositionYCorrection = self.y_PID.getCorrection(self.Y_platform, Y_gps, self.time_step)

        # Pitch, roll, yaw correction
        pitchCorrection = imuPitchCorrection + PositionYCorrection
        rollCorrection  = imuRollCorrection + PositionYCorrection
        yawCorrection = self.yaw_IMU_PID.getCorrection(yaw, 0, self.time_step)

        return [altitudeCorrection, pitchCorrection, rollCorrection, yawCorrection]


     def Stabilizing(self, roll, pitch, yaw, X_gps, Y_gps, Z_gps):
        # 1) roll, pitch and yaw should be roughly zero
        # 2) keep the position

        # Get PID altitude correction
        altitudeCorrection = self.altitude_PID.getCorrection(Z_gps, self.Init_Altitude, self.time_step)

        # PID control for pitch, roll based on angles from Inertial Measurement Unit (IMU)
        imuPitchCorrection = self.pitch_Stability_PID.getCorrection(pitch, self.time_step)
        imuRollCorrection  = self.roll_Stability_PID.getCorrection(-roll, self.time_step)

        # PID control for horizontal position

        PositionXCorrection = self.x_PID.getCorrection(self.Init_GPS_X, X_gps, self.time_step)
        PositionYCorrection = self.y_PID.getCorrection(self.Init_GPS_Y, Y_gps, self.time_step)

        # Pitch, roll, yaw correction
        pitchCorrection = imuPitchCorrection + PositionYCorrection
        rollCorrection  = imuRollCorrection + PositionYCorrection
        yawCorrection = self.yaw_IMU_PID.getCorrection(yaw, 0, self.time_step)

        return [altitudeCorrection, pitchCorrection, rollCorrection, yawCorrection]

     def Yawing(self, roll, pitch, yaw, X_gps, Y_gps, Z_gps,yaw_desired):
        # 1) both roll and pitch should be roughly zero
        # 2) keep the position
        # 3) rotate the yaw to the desired angle

        # Get PID altitude correction if indicated
        altitudeCorrection = self.altitude_PID.getCorrection(Z_gps, self.Init_Altitude, self.time_step)


        # PID control for pitch, roll based on angles from Inertial Measurement Unit (IMU)
        imuPitchCorrection = self.pitch_Stability_PID.getCorrection(pitch, self.time_step)
        imuRollCorrection  = self.roll_Stability_PID.getCorrection(-roll, self.time_step)

        # PID control for horizontal position
        # the rotation matrix, based on direction cosine
        R1=math.cos(yaw)
        R2=math.sin(yaw)
        R3=-math.sin(yaw)
        R4=math.cos(yaw)

        # tranfer the desired position in the Earth coordinate to the body coordinate

        rela_pos1=self.Init_GPS_X-X_gps
        rela_pos2=self.Init_GPS_Y-Y_gps
        desired_pos={}
        desired_pos1=R1*rela_pos1+R2*rela_pos2
        desired_pos2=R3*rela_pos1+R4*rela_pos2

        PositionXCorrection = self.x_PID.getCorrection(desired_pos1, X_gps, self.time_step)
        PositionYCorrection = self.y_PID.getCorrection(desired_pos2, Y_gps, self.time_step)

        # Pitch, roll, yaw correction
        pitchCorrection = imuPitchCorrection + PositionYCorrection
        rollCorrection  = imuRollCorrection + PositionYCorrection

        # need to handle the issue when yaw equals pi
        self.yaw_IMU_PID.t += 1
        target_yaw_timestep = PathOpt(self.T_p, self.Init_yaw, yaw_desired, self.yaw_IMU_PID.t, self.time_step)

        yawCorrection = self.yaw_IMU_PID.getCorrection(yaw, target_yaw_timestep, self.time_step)

        return [altitudeCorrection, pitchCorrection, rollCorrection, yawCorrection]


     def PointFinding(self, roll, pitch, yaw, X_gps, Y_gps, Z_gps,X_desired,Y_desired,Z_desired):
        # 1) yaw to zero
        # 2) move the drone to a desired GPS position
        # 3) the path could be optimized vertically and horizontally
        # optimize the altitude path
        self.altitude_PID.t += 1
        target_altitude_timestep = PathOpt(self.T_p, self.Init_Altitude, self.Z_platform, self.altitude_PID.t, self.time_step)

        # Get PID altitude correction if indicated
        altitudeCorrection = self.altitude_PID.getCorrection(Z_gps, target_altitude_timestep, self.time_step)

        # PID control for pitch, roll based on angles from Inertial Measurement Unit (IMU)
        imuPitchCorrection = self.pitch_Stability_PID.getCorrection(pitch, self.time_step)
        imuRollCorrection  = self.roll_Stability_PID.getCorrection(-roll, self.time_step)

        # PID control for horizontal position

        PositionXCorrection = self.x_PID.getCorrection(self.X_platform, X_gps, self.time_step)
        PositionYCorrection = self.y_PID.getCorrection(self.Y_platform, Y_gps, self.time_step)

        # Pitch, roll, yaw correction
        pitchCorrection = imuPitchCorrection + PositionYCorrection
        rollCorrection  = imuRollCorrection + PositionYCorrection
        yawCorrection = self.yaw_IMU_PID.getCorrection(yaw, 0, self.time_step)

        return [altitudeCorrection, pitchCorrection, rollCorrection, yawCorrection]
     def PathFollowing(self, ):


if __name__ == '__main__':
    main()
