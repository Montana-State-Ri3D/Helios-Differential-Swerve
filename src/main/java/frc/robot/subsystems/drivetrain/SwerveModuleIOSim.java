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

    private final DCMotor bottomMotor;
    private final DCMotor topMotor;

    private double bottomPower;
    private double topPower;

    private final DCMotorSim bottomMotorSim;
    private final DCMotorSim topMotorSim;

    private final double momentjKgMetersSquared = 0.05;

    public SwerveModuleIOSim() {

        bottomMotor = DCMotor.getNEO(1);
        topMotor = DCMotor.getNEO(1);
        bottomMotorSim = new DCMotorSim(bottomMotor, 10.0, momentjKgMetersSquared);
        topMotorSim = new DCMotorSim(topMotor, 10.0, momentjKgMetersSquared);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {

        bottomMotorSim.update(kDt);
        topMotorSim.update(kDt);

        inputs.bottomAngleRad = bottomMotorSim.getAngularPositionRad();
        inputs.bottomAngularVelocityRadPerSec = bottomMotorSim.getAngularVelocityRadPerSec();
        inputs.bottomAppliedPower = bottomPower;
        inputs.bottomCurrentDrawAmps = bottomMotorSim.getCurrentDrawAmps();

        inputs.topAngleRad = topMotorSim.getAngularPositionRad();
        inputs.topAngularVelocityRadPerSec = topMotorSim.getAngularVelocityRadPerSec();
        inputs.topAppliedPower = topPower;
        inputs.topCurrentDrawAmps = topMotorSim.getCurrentDrawAmps();        

        inputs.absoluteAngleRad = (bottomMotorSim.getAngularPositionRad()/STEER_RADIO + topMotorSim.getAngularPositionRad()/STEER_RADIO)/2.0;
        inputs.absoluteAngularVelocityRadPerSec = (bottomMotorSim.getAngularVelocityRadPerSec()/STEER_RADIO + topMotorSim.getAngularVelocityRadPerSec()/STEER_RADIO)/2.0;

        inputs.wheelAngalRad = (bottomMotorSim.getAngularPositionRad()/DRIVE_RADIO - topMotorSim.getAngularPositionRad()/DRIVE_RADIO)/2.0;
        inputs.wheelAngularVelocityRadPerSec = (bottomMotorSim.getAngularVelocityRadPerSec()/DRIVE_RADIO - topMotorSim.getAngularVelocityRadPerSec()/DRIVE_RADIO)/2.0;
        inputs.wheelSpeedMPerSec = inputs.wheelAngularVelocityRadPerSec * (WHEEL_DIAMETER_METERS/2.0);
        inputs.wheelDistanceM = inputs.wheelAngalRad * (WHEEL_DIAMETER_METERS/2.0);
    }

    @Override
    public void setVoltage(double bottomPower, double topPower) {

        if (Robot.isEnabled) {
            this.bottomPower = bottomPower;
            this.topPower = topPower;

            topMotorSim.setInputVoltage(topPower);
            bottomMotorSim.setInputVoltage(bottomPower);
        } else {
            this.bottomPower = 0;
            this.topPower = 0;

            topMotorSim.setInputVoltage(0);
            bottomMotorSim.setInputVoltage(0);
        }

    }
}
