package frc.robot.utility;

import edu.wpi.first.wpilibj.RobotBase;

public enum RobotIdentity {
    HELIOS_TEST_BOT,
    SIMULATION,
    HELIOS_TEST_MODUAL;

    /**
     * Gets the identity of the robot.
     *
     * @return The detected identity of the robot.
     */
    public static RobotIdentity getIdentity() {
        // When we're running on a real robot we'll base the identity on its MAC address
        if (RobotBase.isReal()) {
            return HELIOS_TEST_MODUAL;
        } else {
            return SIMULATION;
        }
    }
}
