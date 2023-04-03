// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDrivetrainCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.utility.ControllerHelper;
import frc.robot.utility.RobotIdentity;
import frc.robot.utility.SubsystemFactory;

import static frc.robot.Constants.*;

public class RobotContainer {

  private DrivetrainSubsystem drivetrainSubsystem;

  private DefaultDrivetrainCommand defaultDrivetrainCommand;

  private RobotIdentity indetity;

  // Creating Controlers
  private final CommandXboxController driveController = new CommandXboxController(DRIVE_CONTROLLER_PORT);

  public RobotContainer(RobotIdentity indetity) {
    this.indetity = indetity;
    createSubsystems();
    createCommands();
    configureBindings();
  }

  private void createSubsystems() {
    drivetrainSubsystem = SubsystemFactory.createDrivetrain(indetity);
  }

  private void createCommands() {
    ControllerHelper driverHelper = new ControllerHelper();

    if (driveController.getHID().isConnected()) {
      defaultDrivetrainCommand = new DefaultDrivetrainCommand(drivetrainSubsystem,
          () -> driverHelper.modifyAxis(-driveController.getLeftY())
              * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
          () -> driverHelper.modifyAxis(-driveController.getLeftX())
              * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
          () -> driverHelper.modifyAxis(-driveController.getRightX())
              * drivetrainSubsystem.getMaxAngularVelocityRadPerSec());
    } else {
      defaultDrivetrainCommand = new DefaultDrivetrainCommand(drivetrainSubsystem, () -> 0.0, () -> 0.0, () -> 0.0);
    }
    drivetrainSubsystem.setDefaultCommand(defaultDrivetrainCommand);

  }

  private void configureBindings() {
    if (driveController.getHID().isConnected()) {
      driveController.start().onTrue(new InstantCommand(() -> drivetrainSubsystem.resetPose(new Pose2d())));

      driveController.x().onTrue(new InstantCommand(() -> defaultDrivetrainCommand.toggleFieldOriented()));

      driveController.back().onTrue(new InstantCommand(
          () -> drivetrainSubsystem.resetPose(
              new Pose2d(drivetrainSubsystem.getPose().getX(), drivetrainSubsystem.getPose().getY(),
                  new Rotation2d()))));
    }

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
