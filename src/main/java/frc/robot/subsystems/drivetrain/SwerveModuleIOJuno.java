package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.*;

public class SwerveModuleIOJuno implements SwerveModuleIO {
    // This value accounts for the wearing down of the tread over the course of a
    // competition, as well as the "squish factor" between
    // the wheels and the carpet. Function may be added down the line to dynamically
    // adjust this value in the field.


    private final CANSparkMax bottomMotor;
    private final CANSparkMax topMotor;
    private final CANCoder steerEncoder;
    private final RelativeEncoder bottomEncoder;
    private final RelativeEncoder topEncoder;
    private final double offset;


    public SwerveModuleIOJuno(
            int bottomMotorID,
            int topMotorID,
            int steerEncoderID,
            double offset) {

        bottomMotor = new CANSparkMax(bottomMotorID, MotorType.kBrushless);
        topMotor = new CANSparkMax(topMotorID, MotorType.kBrushless);
        steerEncoder = new CANCoder(steerEncoderID);

        this.offset = offset;

        this.bottomEncoder = bottomMotor.getEncoder();
        this.topEncoder = topMotor.getEncoder();

        bottomMotor.setSmartCurrentLimit(40);
        topMotor.setSmartCurrentLimit(40);

        bottomMotor.setControlFramePeriodMs((int)kDt*1000);
        topMotor.setControlFramePeriodMs((int)kDt*1000);
        steerEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData,(int)kDt*1000);

        steerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

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

        inputs.absoluteAngleRad = Math.toRadians(steerEncoder.getPosition())+offset;
        inputs.absoluteAngularVelocityRadPerSec = Math.toRadians(steerEncoder.getVelocity());

        inputs.wheelAngalRad = Units.degreesToRadians(steerEncoder.getAbsolutePosition());
        inputs.wheelAngularVelocityRadPerSec = Units.degreesToRadians(steerEncoder.getVelocity());
        inputs.wheelSpeedMPerSec = inputs.wheelAngularVelocityRadPerSec * (WHEEL_DIAMETER_METERS/2.0);
        inputs.wheelDistanceM = inputs.wheelAngalRad * (WHEEL_DIAMETER_METERS/2.0);

        inputs.topAngleRad = topEncoder.getPosition();
        inputs.topAngularVelocityRadPerSec = topEncoder.getVelocity();
        inputs.topAppliedPower = topMotor.getAppliedOutput();
        inputs.topCurrentDrawAmps = topMotor.getOutputCurrent();
    }

    @Override
    public void setVoltage(double bottomPower, double topPower) {
        bottomMotor.setVoltage(bottomPower);
        topMotor.setVoltage(topPower);
    }
}
