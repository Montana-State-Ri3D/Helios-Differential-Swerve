// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.Constants.*;

public class RobotContainer {

   // Creating Controlers
   @SuppressWarnings({ "unused" })
   private final CommandXboxController driveController = new CommandXboxController(DRIVE_CONTROLLER_PORT);
   @SuppressWarnings({ "unused" })
   private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);
   @SuppressWarnings({ "unused" })
   private final CommandXboxController testController = new CommandXboxController(TEST_CONTROLLER_PORT);

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
