// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class SetModuleZero extends CommandBase {

  private final DrivetrainSubsystem drivetrain;

  /** Creates a new SetModuleZero. */
  public SetModuleZero(DrivetrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setManualModuleStateEnable(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[] {
        new SwerveModuleState(0.0, new Rotation2d(0.0)),
        new SwerveModuleState(0.0, new Rotation2d(0.0)),
        new SwerveModuleState(0.0, new Rotation2d(0.0)),
        new SwerveModuleState(0.0, new Rotation2d(0.0)),
    };

    drivetrain.setManualModuleState(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setManualModuleStateEnable(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
