package frc.robot.utility;

import static frc.robot.Constants.*;


public final class SubsystemFactory {
    public static ArmSubsystem createArm(RobotIdentity identity) {
        switch (identity) {
            case SIMULATION:
                return new ArmSubsystem(new ArmIOSim());
            default:
                return new ArmSubsystem(new ArmIO() {
                });
        }
    }
}

