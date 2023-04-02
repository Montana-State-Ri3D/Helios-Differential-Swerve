package frc.robot.utility;

import static frc.robot.Constants.*;

import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.SwerveModuleIOJuno;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;

public final class SubsystemFactory {

        public static DrivetrainSubsystem createDrivetrain(RobotIdentity identity) {

                DrivetrainSubsystem drivetrain;

                switch (identity) {
                        case JUNO_TEST_BOT:
                                drivetrain = new DrivetrainSubsystem(
                                                new GyroIOPigeon2(DRIVETRAIN_PIGEON_ID),
                                                new SwerveModuleIOJuno(
                                                                DRIVETRAIN_FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                                                DRIVETRAIN_FRONT_LEFT_MODULE_STEER_MOTOR,
                                                                DRIVETRAIN_FRONT_LEFT_MODULE_STEER_ENCODER,
                                                                FRONT_LEFT_MODULE_STEER_OFFSET),
                                                new SwerveModuleIOJuno(
                                                                DRIVETRAIN_FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                                                DRIVETRAIN_FRONT_RIGHT_MODULE_STEER_MOTOR,
                                                                DRIVETRAIN_FRONT_RIGHT_MODULE_STEER_ENCODER,
                                                                FRONT_RIGHT_MODULE_STEER_OFFSET),
                                                new SwerveModuleIOJuno(
                                                                DRIVETRAIN_BACK_LEFT_MODULE_DRIVE_MOTOR,
                                                                DRIVETRAIN_BACK_LEFT_MODULE_STEER_MOTOR,
                                                                DRIVETRAIN_BACK_LEFT_MODULE_STEER_ENCODER,
                                                                BACK_LEFT_MODULE_STEER_OFFSET),
                                                new SwerveModuleIOJuno(
                                                                DRIVETRAIN_BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                                                DRIVETRAIN_BACK_RIGHT_MODULE_STEER_MOTOR,
                                                                DRIVETRAIN_BACK_RIGHT_MODULE_STEER_ENCODER,
                                                                BACK_RIGHT_MODULE_STEER_OFFSET));
                                break;
                        case JUNO_TEST_MODUAL:
                                drivetrain = new DrivetrainSubsystem(
                                                new GyroIOSim(),
                                                new SwerveModuleIOJuno(
                                                                DRIVETRAIN_FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                                                DRIVETRAIN_FRONT_LEFT_MODULE_STEER_MOTOR,
                                                                DRIVETRAIN_FRONT_LEFT_MODULE_STEER_ENCODER,
                                                                FRONT_LEFT_MODULE_STEER_OFFSET),
                                                new SwerveModuleIOSim(),
                                                new SwerveModuleIOSim(),
                                                new SwerveModuleIOSim());
                                break;
                        case SIMULATION:
                                drivetrain = new DrivetrainSubsystem(
                                                new GyroIOSim(),
                                                new SwerveModuleIOSim(),
                                                new SwerveModuleIOSim(),
                                                new SwerveModuleIOSim(),
                                                new SwerveModuleIOSim()

                                );
                                break;
                        default:
                                return new DrivetrainSubsystem(
                                                new GyroIO() {
                                                },
                                                new SwerveModuleIO() {
                                                },
                                                new SwerveModuleIO() {
                                                },
                                                new SwerveModuleIO() {
                                                },
                                                new SwerveModuleIO() {
                                                });
                }

                return drivetrain;
        }
}
