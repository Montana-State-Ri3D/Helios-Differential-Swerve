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
import frc.robot.utility.AutoCommandChooser;
import frc.robot.utility.AutoCommandFactory;
import frc.robot.utility.ControllerHelper;
import frc.robot.utility.RobotIdentity;
import frc.robot.utility.SubsystemFactory;

import static frc.robot.Constants.*;

public class RobotContainer {

  private DrivetrainSubsystem drivetrainSubsystem;

  private DefaultDrivetrainCommand defaultDrivetrainCommand;

  private RobotIdentity indetity;
  private AutoCommandChooser autoChooser;

  // Creating Controlers
  private final CommandXboxController driveController = new CommandXboxController(DRIVE_CONTROLLER_PORT);

  public RobotContainer(RobotIdentity indetity) {
    this.indetity = indetity;

    createSubsystems();
    createCommands();
    configureBindings();
    setupAutoChooser();
  }

  private void createSubsystems() {
    drivetrainSubsystem = SubsystemFactory.createDrivetrain(indetity);
  }

  private void createCommands() {
    ControllerHelper driverHelper = new ControllerHelper();
    AutoCommandFactory.init(drivetrainSubsystem);

    defaultDrivetrainCommand = new DefaultDrivetrainCommand(drivetrainSubsystem,
        () -> driverHelper.modifyAxis(-driveController.getLeftY())
            * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
        () -> driverHelper.modifyAxis(-driveController.getLeftX())
            * drivetrainSubsystem.getMaxTranslationalVelocityMetersPerSecond(),
        () -> driverHelper.modifyAxis(-driveController.getRightX())
            * drivetrainSubsystem.getMaxAngularVelocityRadPerSec());

    drivetrainSubsystem.setDefaultCommand(defaultDrivetrainCommand);

  }

  private void configureBindings() {

    driveController.start().onTrue(new InstantCommand(() -> drivetrainSubsystem.resetPose(new Pose2d())));

    driveController.x().onTrue(new InstantCommand(() -> defaultDrivetrainCommand.toggleFieldOriented()));

    driveController.back().onTrue(new InstantCommand(
        () -> drivetrainSubsystem.resetPose(
            new Pose2d(drivetrainSubsystem.getPose().getX(), drivetrainSubsystem.getPose().getY(),
                new Rotation2d()))));

  }

  private void setupAutoChooser() {
    autoChooser = new AutoCommandChooser();

    // Register all the supported auto commands
    autoChooser.registerDefaultCreator("Do Nothing", () -> AutoCommandFactory.createNullAuto());
    autoChooser.registerCreator("Test Path", () -> AutoCommandFactory.createTestPath());
    autoChooser.registerCreator("Cool Path", () -> AutoCommandFactory.createNoBumpSide3Auto());
    autoChooser.registerCreator("Drive Straight", () -> AutoCommandFactory.createDriveStraignt());

    // Setup the chooser in shuffleboard
    autoChooser.setup("Driver", 0, 0, 3, 1);
}

/**
 * Use this to pass the autonomous command to the main {@link Robot} class.
 *
 * @return the command to run in autonomous
 */
public Command getAutonomousCommand() {
    return autoChooser.getAutonomousCommand();
}
}
