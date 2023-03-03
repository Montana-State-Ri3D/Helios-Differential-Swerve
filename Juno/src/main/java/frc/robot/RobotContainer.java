// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    createSubsystems();
    createCommands();
    configureBindings();
  }
  private void createCommands(){

  }
  private void createSubsystems(){

  }
  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
