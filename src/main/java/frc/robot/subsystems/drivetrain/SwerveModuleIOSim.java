package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.Robot;

import static frc.robot.Constants.*;

public class SwerveModuleIOSim implements SwerveModuleIO {
    // This value accounts for the wearing down of the tread over the course of a
    // competition, as well as the "squish factor" between
    // the wheels and the carpet. Function may be added down the line to dynamically
    // adjust this value in the field.

    private final EncoderSim steerEncoder;

    private final DCMotor leftMotor;
    private final DCMotor rightMotor;

    private double leftPower;
    private double rightPower;

    private final DCMotorSim leftMotorSim;
    private final DCMotorSim rightMotorSim;

    private final double momentjKgMetersSquared = 10;

    public SwerveModuleIOSim() {

        leftMotor = DCMotor.getNEO(1);
        rightMotor = DCMotor.getNEO(1);
        leftMotorSim = new DCMotorSim(leftMotor, 1.0, momentjKgMetersSquared);
        rightMotorSim = new DCMotorSim(rightMotor, 1.0, momentjKgMetersSquared);

        steerEncoder = EncoderSim.createForIndex(1);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {

        leftMotorSim.update(kDt);
        rightMotorSim.update(kDt);

        steerEncoder.setDistance((leftMotorSim.getAngularPositionRad() - rightMotorSim.getAngularPositionRad()) / 2.0);
        steerEncoder.setRate(
                (leftMotorSim.getAngularVelocityRadPerSec() - rightMotorSim.getAngularVelocityRadPerSec()) / 2.0);

        inputs.leftAngleRad = leftMotorSim.getAngularPositionRad();
        inputs.leftAngularVelocityRadPerSec = leftMotorSim.getAngularVelocityRadPerSec();
        inputs.leftAppliedPower = leftPower;
        inputs.leftCurrentDrawAmps = leftMotorSim.getCurrentDrawAmps();

        inputs.absoluteAngleRad = Math.toRadians(steerEncoder.getDistance());
        inputs.absoluteAngularVelocityRadPerSec = Math.toRadians(steerEncoder.getRate());

        inputs.wheelAngalRad = (leftMotorSim.getAngularPositionRad() + rightMotorSim.getAngularPositionRad()) / 2.0;
        inputs.wheelAngularVelocityRadPerSec = (leftMotorSim.getAngularVelocityRadPerSec()
                + rightMotorSim.getAngularVelocityRadPerSec()) / 2.0;

        inputs.rightAngleRad = rightMotorSim.getAngularPositionRad();
        inputs.rightAngularVelocityRadPerSec = rightMotorSim.getAngularVelocityRadPerSec();
        inputs.rightAppliedPower = rightPower;
        inputs.rightCurrentDrawAmps = rightMotorSim.getCurrentDrawAmps();
    }

    @Override
    public void setSpeeds(double leftPower, double rightPower) {

        if (Robot.isEnabled) {
            this.leftPower = leftPower;
            this.rightPower = rightPower;

            rightMotorSim.setInput(rightPower);
            leftMotorSim.setInput(leftPower);
        } else {
            this.leftPower = 0;
            this.rightPower = 0;

            rightMotorSim.setInput(0);
            leftMotorSim.setInput(0);
        }

    }
}
