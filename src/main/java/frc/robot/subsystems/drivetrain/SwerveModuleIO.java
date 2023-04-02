package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOInputs {
        public double leftAngleRad;
        public double leftAngularVelocityRadPerSec;
        public double leftAppliedPower;
        public double leftCurrentDrawAmps;

        public double absoluteAngleRad;
        public double absoluteAngularVelocityRadPerSec;
        public double wheelAngalRad;
        public double wheelAngularVelocityRadPerSec;
        public double wheelSpeedMPerSec;
        public double wheelDistanceM;

        public double rightAngleRad;
        public double rightAngularVelocityRadPerSec;
        public double rightAppliedPower;
        public double rightCurrentDrawAmps;
    }
    
    default void updateInputs(SwerveModuleIOInputs inputs) {
    }

    default void setSpeeds(double leftPower,double rightPower) {
    }
}
