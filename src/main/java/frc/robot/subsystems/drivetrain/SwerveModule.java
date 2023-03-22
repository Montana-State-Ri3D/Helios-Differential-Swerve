package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final String name;
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public SwerveModule(String name, SwerveModuleIO io) {
        this.name = name;
        this.io = io;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drivetrain/" + name, inputs);
    }

    public void setTargetState(SwerveModuleState state) {
        io.drive(state.speedMetersPerSecond, state.angle.getRadians());
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
                inputs.drivePositionMeters,
                new Rotation2d(inputs.absoluteAngleRad)
        );
    }
}
