// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int TEST_CONTROLLER_PORT = 2;

    public static final int PDP_CAN_ID = 0;

    public static final double DRIVE_RADIO = 1.0;
    public static final double STEER_RADIO = 1.0;
    public static final double WHEEL_NOMINAL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    public static final double TREADWEAR = Units.inchesToMeters(0.0);


    public static final int DRIVETRAIN_FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
    public static final int DRIVETRAIN_BACK_LEFT_MODULE_DRIVE_MOTOR = 3;
    public static final int DRIVETRAIN_BACK_RIGHT_MODULE_DRIVE_MOTOR = 4;

    public static final int DRIVETRAIN_FRONT_LEFT_MODULE_STEER_MOTOR = 5;
    public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_STEER_MOTOR = 6;
    public static final int DRIVETRAIN_BACK_LEFT_MODULE_STEER_MOTOR = 7;
    public static final int DRIVETRAIN_BACK_RIGHT_MODULE_STEER_MOTOR = 8;

    public static final int DRIVETRAIN_FRONT_LEFT_MODULE_STEER_ENCODER = 9;
    public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final int DRIVETRAIN_BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final int DRIVETRAIN_BACK_RIGHT_MODULE_STEER_ENCODER = 12;

    public static final int DRIVETRAIN_PIGEON_ID = 13;

    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(0);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(0);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(0);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(0);
}
