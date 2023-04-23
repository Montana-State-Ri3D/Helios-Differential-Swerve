package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
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

    private final double momentjKgMetersSquared = 0.0001;

    public SwerveModuleIOSim() {

        bottomMotor = DCMotor.getNEO(1);
        topMotor = DCMotor.getNEO(1);
        bottomMotorSim = new DCMotorSim(bottomMotor, 1.0, momentjKgMetersSquared);
        topMotorSim = new DCMotorSim(topMotor, 1.0, momentjKgMetersSquared);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {

        bottomMotorSim.update(kDt);
        topMotorSim.update(kDt);

        inputs.bottomAngleRad = bottomMotorSim.getAngularPositionRad();
        inputs.bottomAngularVelocityRadPerSec = bottomMotorSim.getAngularVelocityRadPerSec();
        inputs.bottomAppliedPower = bottomPower;
        inputs.bottomCurrentDrawAmps = bottomMotorSim.getCurrentDrawAmps();
        inputs.bottomTemp = 0;

        inputs.topAngleRad = topMotorSim.getAngularPositionRad();
        inputs.topAngularVelocityRadPerSec = topMotorSim.getAngularVelocityRadPerSec();
        inputs.topAppliedPower = topPower;
        inputs.topCurrentDrawAmps = topMotorSim.getCurrentDrawAmps();  
        inputs.topTemp = 0;      

        inputs.absoluteAngleRad = MathUtil.inputModulus(((topMotorSim.getAngularPositionRad() - bottomMotorSim.getAngularPositionRad())*STEER_RADIO)/(2.0), -Math.PI, Math.PI);
        inputs.absoluteAngularVelocityRadPerSec = ((topMotorSim.getAngularVelocityRadPerSec() - bottomMotorSim.getAngularVelocityRadPerSec())*STEER_RADIO)/(2.0);

        inputs.wheelAngalRad = ((bottomMotorSim.getAngularPositionRad() + topMotorSim.getAngularPositionRad())*DRIVE_RADIO)/(2.0);
        inputs.wheelAngularVelocityRadPerSec = ((bottomMotorSim.getAngularVelocityRadPerSec() + topMotorSim.getAngularVelocityRadPerSec())*DRIVE_RADIO)/(2.0);
        
        inputs.wheelSpeedMPerSec = inputs.wheelAngularVelocityRadPerSec * (WHEEL_DIAMETER_METERS/2.0);
        inputs.wheelDistanceM = inputs.wheelAngalRad * (WHEEL_DIAMETER_METERS/2.0);
    }

    @Override
    public void setVoltages(double bottomPower, double topPower) {

        if (Robot.isEnabled) {
            this.bottomPower = bottomPower;
            this.topPower = topPower;

            topMotorSim.setInputVoltage(this.topPower);
            bottomMotorSim.setInputVoltage(this.bottomPower);
        } else {
            this.bottomPower = 0;
            this.topPower = 0;

            topMotorSim.setInputVoltage(0);
            bottomMotorSim.setInputVoltage(0);
        }

    }
}
