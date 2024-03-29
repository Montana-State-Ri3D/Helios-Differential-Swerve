package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DefaultDrivetrainCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier xVelocitySupplier;
    private final DoubleSupplier yVelocitySupplier;
    private final DoubleSupplier angularVelocitySupplier;
    private boolean isFieldOriented = false;

    public DefaultDrivetrainCommand(DrivetrainSubsystem drivetrain,
            DoubleSupplier xVelocitySupplier,
            DoubleSupplier yVelocitySupplier,
            DoubleSupplier angularVelocitySupplier) {
        this.drivetrain = drivetrain;
        this.xVelocitySupplier = xVelocitySupplier;
        this.yVelocitySupplier = yVelocitySupplier;
        this.angularVelocitySupplier = angularVelocitySupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        ChassisSpeeds chassisVelocity;
        if (isFieldOriented) {
            chassisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xVelocitySupplier.getAsDouble(),
                    yVelocitySupplier.getAsDouble(),
                    angularVelocitySupplier.getAsDouble(),
                    drivetrain.getPose().getRotation());
        } else {
            chassisVelocity = new ChassisSpeeds(
                    xVelocitySupplier.getAsDouble(),
                    yVelocitySupplier.getAsDouble(),
                    angularVelocitySupplier.getAsDouble());
        }
        drivetrain.setTargetChassisVelocity(chassisVelocity);
    }

    public void fieldOriented(boolean isFieldOriented) {
        this.isFieldOriented = isFieldOriented;
    }

    public void toggleFieldOriented() {
        this.isFieldOriented = !isFieldOriented;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setTargetChassisVelocity(new ChassisSpeeds());
    }
}
