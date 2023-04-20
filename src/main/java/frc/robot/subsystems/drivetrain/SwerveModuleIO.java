package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOInputs {
        public double bottomAngleRad;
        public double bottomAngularVelocityRadPerSec;
        public double bottomAppliedPower;
        public double bottomCurrentDrawAmps;
        public double bottomTemp;

        public double absoluteAngleRad;
        public double absoluteAngularVelocityRadPerSec;
        public double wheelAngalRad;
        public double wheelAngularVelocityRadPerSec;
        public double wheelSpeedMPerSec;
        public double wheelDistanceM;

        public double topAngleRad;
        public double topAngularVelocityRadPerSec;
        public double topAppliedPower;
        public double topCurrentDrawAmps;
        public double topTemp;
    }

    default void updateInputs(SwerveModuleIOInputs inputs) {
    }

    default void setVoltages(double bottomPower, double topPower) {
    }
}
