package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.*;

public class SwerveModuleIOHelios implements SwerveModuleIO {
    // This value accounts for the wearing down of the tread over the course of a
    // competition, as well as the "squish factor" between
    // the wheels and the carpet. Function may be added down the line to dynamically
    // adjust this value in the field.


    private final CANSparkMax bottomMotor;
    private final CANSparkMax topMotor;
    private final CANCoder steerEncoder;
    private final RelativeEncoder bottomEncoder;
    private final RelativeEncoder topEncoder;


    public SwerveModuleIOHelios(
            int bottomMotorID,
            int topMotorID,
            int steerEncoderID,
            double offset) {

        bottomMotor = new CANSparkMax(bottomMotorID, MotorType.kBrushless);
        topMotor = new CANSparkMax(topMotorID, MotorType.kBrushless);
        steerEncoder = new CANCoder(steerEncoderID);

        steerEncoder.configMagnetOffset(Math.toDegrees(offset));

        IdleMode idleMode = IdleMode.kBrake;

        bottomMotor.setIdleMode(idleMode);
        topMotor.setIdleMode(idleMode);

        this.bottomEncoder = bottomMotor.getEncoder();
        this.topEncoder = topMotor.getEncoder();

        bottomEncoder.setPositionConversionFactor(Math.PI*2.0);
        topEncoder.setPositionConversionFactor(Math.PI*2.0);

        bottomEncoder.setVelocityConversionFactor((Math.PI*2.0)/60.0);
        topEncoder.setVelocityConversionFactor((Math.PI*2.0)/60.0);

        bottomMotor.setSmartCurrentLimit(DRIVETRAIN_MAX_CURRENT);
        topMotor.setSmartCurrentLimit(DRIVETRAIN_MAX_CURRENT);

        bottomMotor.setControlFramePeriodMs((int)kDt*1000);
        topMotor.setControlFramePeriodMs((int)kDt*1000);

        bottomEncoder.setPosition(0);
        topEncoder.setPosition(0);

        steerEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData,(int)kDt*1000);

        steerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        // Workaround so that we always read a valid angle from the steer encoder when
        // setting up the steer motor.
        // Avoid using Thread.sleep and replace with an actual way to check if the steer
        // encoder has received valid data
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            System.err.println("OOPS");
        }

    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.bottomAngleRad = bottomEncoder.getPosition();
        inputs.bottomAngularVelocityRadPerSec = bottomEncoder.getVelocity();
        inputs.bottomAppliedPower = bottomMotor.getAppliedOutput();
        inputs.bottomCurrentDrawAmps = bottomMotor.getOutputCurrent();
        inputs.bottomTemp = bottomMotor.getMotorTemperature();

        inputs.topAngleRad = topEncoder.getPosition();
        inputs.topAngularVelocityRadPerSec = topEncoder.getVelocity();
        inputs.topAppliedPower = topMotor.getAppliedOutput();
        inputs.topCurrentDrawAmps = topMotor.getOutputCurrent();
        inputs.topTemp = topMotor.getMotorTemperature();

        inputs.absoluteAngleRad = (Units.degreesToRadians(steerEncoder.getAbsolutePosition()));
        inputs.absoluteAngularVelocityRadPerSec = Units.degreesToRadians(steerEncoder.getVelocity());

        //inputs.absoluteAngleRad = MathUtil.inputModulus(((topEncoder.getPosition() - bottomEncoder.getPosition())*STEER_RADIO)/(2.0),-Math.PI,Math.PI);
        //inputs.absoluteAngularVelocityRadPerSec = ((topEncoder.getVelocity() - bottomEncoder.getVelocity())*STEER_RADIO)/(2.0);

        inputs.wheelAngalRad = ((bottomEncoder.getPosition() + topEncoder.getPosition())*DRIVE_RADIO)/(2.0);
        inputs.wheelAngularVelocityRadPerSec = ((bottomEncoder.getVelocity() + topEncoder.getVelocity())*DRIVE_RADIO)/(2.0);
        
        inputs.wheelSpeedMPerSec = inputs.wheelAngularVelocityRadPerSec * (WHEEL_DIAMETER_METERS/2.0);
        inputs.wheelDistanceM = inputs.wheelAngalRad * (WHEEL_DIAMETER_METERS/2.0);
    }

    @Override
    public void setVoltages(double bottomPower, double topPower) {
        bottomMotor.setVoltage(bottomPower);
        topMotor.setVoltage(topPower);
    }
}
