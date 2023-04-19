// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int TEST_CONTROLLER_PORT = 2;

    public static final int PDP_CAN_ID = 0;

    public static final double DRIVE_RADIO = (10.0 / 84.0) * (54.0 / 12.0) * (15.0 / 45.0);
    public static final double STEER_RADIO = (10.0 / 84.0);

    public static final double TRANSLATIONAN_FREE_SPEED = 5.9436;

    public static final double MOTOR_FREE_SPEED = 5676;

    public static final double WHEEL_NOMINAL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    public static final double TREADWEAR = Units.inchesToMeters(0.0);
    public static final double WHEEL_DIAMETER_METERS = WHEEL_NOMINAL_DIAMETER_METERS - TREADWEAR;

    public static final int DRIVETRAIN_FRONT_LEFT_MODULE_BOTTOM = 1;
    public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_BOTTOM = 2;
    public static final int DRIVETRAIN_BACK_LEFT_MODULE_BOTTOM = 3;
    public static final int DRIVETRAIN_BACK_RIGHT_MODULE_BOTTOM = 4;

    public static final int DRIVETRAIN_FRONT_LEFT_MODULE_TOP = 5;
    public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_TOP = 6;
    public static final int DRIVETRAIN_BACK_LEFT_MODULE_TOP = 7;
    public static final int DRIVETRAIN_BACK_RIGHT_MODULE_TOP = 8;

    public static final int DRIVETRAIN_FRONT_LEFT_MODULE_STEER_ENCODER = 9;
    public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final int DRIVETRAIN_BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final int DRIVETRAIN_BACK_RIGHT_MODULE_STEER_ENCODER = 12;

    public static final int DRIVETRAIN_PIGEON_ID = 13;

    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(0);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(0);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(0);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(0);

    // update rate of our modules 5ms.
    public static final double kDt = 0.02;

    public static final double VOLTAGE = 12;

    public static final double FEED_FORWARD = VOLTAGE / (MOTOR_FREE_SPEED / DRIVE_RADIO);

    public static final double INERTIA_WHEEL = 0.005;
    public static final double INERTIA_STEER = 0.004;

    // This is for Kalman filter which isn't used for azimuth angle due to angle
    // wrapping.
    // Model noise are assuming that our model isn't as accurate as our sensors.
    public static final double MODEL_AZIMUTH_ANGLE_NOISE = .1; // radians
    public static final double MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 5.0; // radians per sec
    public static final double MODEL_WHEEL_ANG_VELOCITY_NOISE = 5.0; // radians per sec

    // Noise from sensors. Falcon With Gearbox causes us to have more uncertainty so
    // we increase
    // the noise.
    public static final double SENSOR_AZIMUTH_ANGLE_NOISE = 0.01; // radians
    public static final double SENSOR_AZIMUTH_ANG_VELOCITY_NOISE = 0.1; // radians per sec
    public static final double SENSOR_WHEEL_ANG_VELOCITY_NOISE = 0.1; // radians per sec

    // A weight for how aggressive each state should be ie. 0.08 radians will try to
    // control the
    // angle more aggressively than the wheel angular velocity.
    public static final double Q_AZIMUTH_ANG_VELOCITY = 1.1; // radians per sec
    public static final double Q_AZIMUTH = 0.08; // radians
    public static final double Q_WHEEL_ANG_VELOCITY = 5; // radians per sec

    public static final double CONTROL_EFFORT = 0.8;

    // Size of the robot chassis in meters
    public static final double WIDTH = Units.feetToMeters(2); // meters
    public static final double LENGTH = Units.feetToMeters(2); // meters

    // Distance of swerve modules from center of robot
    public static final double SWERVE_FB_POS = LENGTH / 2.0;
    public static final double SWERVE_LR_POS = WIDTH / 2.0;

    // Position vectors for the swerve module kinematics
    // i.e. location of each swerve module from center of robot
    public static final Translation2d FROUNT_LEFT = new Translation2d(SWERVE_FB_POS, SWERVE_LR_POS); // +,+
    public static final Translation2d BACK_LEFT = new Translation2d(-SWERVE_FB_POS, SWERVE_LR_POS); // -,+
    public static final Translation2d BACK_RIGHT = new Translation2d(-SWERVE_FB_POS, -SWERVE_LR_POS); // -,-
    public static final Translation2d FROUNT_RIGHT = new Translation2d(SWERVE_FB_POS, -SWERVE_LR_POS); // +,-
}
