package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

import static frc.robot.Constants.*;

public class SwerveModuleIOSim implements SwerveModuleIO {
    // This value accounts for the wearing down of the tread over the course of a
    // competition, as well as the "squish factor" between
    // the wheels and the carpet. Function may be added down the line to dynamically
    // adjust this value in the field.

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
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {

        leftMotorSim.update(kDt);
        rightMotorSim.update(kDt);

        inputs.leftAngleRad = leftMotorSim.getAngularPositionRad();
        inputs.leftAngularVelocityRadPerSec = leftMotorSim.getAngularVelocityRadPerSec();
        inputs.leftAppliedPower = leftPower;
        inputs.leftCurrentDrawAmps = leftMotorSim.getCurrentDrawAmps();

        inputs.rightAngleRad = rightMotorSim.getAngularPositionRad();
        inputs.rightAngularVelocityRadPerSec = rightMotorSim.getAngularVelocityRadPerSec();
        inputs.rightAppliedPower = rightPower;
        inputs.rightCurrentDrawAmps = rightMotorSim.getCurrentDrawAmps();        

        inputs.absoluteAngleRad = (leftMotorSim.getAngularPositionRad()/STEER_RADIO + rightMotorSim.getAngularPositionRad()/STEER_RADIO)/2.0;
        inputs.absoluteAngularVelocityRadPerSec = (leftMotorSim.getAngularVelocityRadPerSec()/STEER_RADIO + rightMotorSim.getAngularVelocityRadPerSec()/STEER_RADIO)/2.0;

        inputs.wheelAngalRad = (leftMotorSim.getAngularPositionRad()/DRIVE_RADIO - rightMotorSim.getAngularPositionRad()/DRIVE_RADIO)/2.0;
        inputs.wheelAngularVelocityRadPerSec = (leftMotorSim.getAngularVelocityRadPerSec()/DRIVE_RADIO - rightMotorSim.getAngularVelocityRadPerSec()/DRIVE_RADIO)/2.0;
        inputs.wheelSpeedMPerSec = inputs.wheelAngularVelocityRadPerSec * (WHEEL_DIAMETER_METERS/2.0);
        inputs.wheelDistanceM = inputs.wheelAngalRad * (WHEEL_DIAMETER_METERS/2.0);
    }

    @Override
    public void setSpeeds(double leftPower, double rightPower) {

        if (Robot.isEnabled) {
            this.leftPower = leftPower;
            this.rightPower = rightPower;

            rightMotorSim.setInputVoltage(rightPower);
            leftMotorSim.setInputVoltage(leftPower);
        } else {
            this.leftPower = 0;
            this.rightPower = 0;

            rightMotorSim.setInputVoltage(0);
            leftMotorSim.setInputVoltage(0);
        }

    }
}
