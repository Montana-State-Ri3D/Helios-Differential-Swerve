package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
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


    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final CANCoder steerEncoder;
    private final RelativeEncoder lefEncoder;
    private final RelativeEncoder righEncoder;


    public SwerveModuleIOJuno(
            int leftMotorID,
            int rightMotorID,
            int steerEncoderID,
            double offset) {

        leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
        steerEncoder = new CANCoder(steerEncoderID);

        this.lefEncoder = leftMotor.getEncoder();
        this.righEncoder = rightMotor.getEncoder();

        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);

        steerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        steerEncoder.configMagnetOffset(Math.toDegrees(offset));

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
        inputs.leftAngleRad = lefEncoder.getPosition();
        inputs.leftAngularVelocityRadPerSec = lefEncoder.getVelocity();
        inputs.leftAppliedPower = leftMotor.getAppliedOutput();
        inputs.leftCurrentDrawAmps = leftMotor.getOutputCurrent();

        inputs.absoluteAngleRad = Math.toRadians(steerEncoder.getPosition());
        inputs.absoluteAngularVelocityRadPerSec = Math.toRadians(steerEncoder.getVelocity());

        inputs.wheelAngalRad = Units.degreesToRadians(steerEncoder.getAbsolutePosition());
        inputs.wheelAngularVelocityRadPerSec = Units.degreesToRadians(steerEncoder.getVelocity());
        inputs.wheelSpeedMPerSec = inputs.wheelAngularVelocityRadPerSec * (WHEEL_DIAMETER_METERS/2.0);
        inputs.wheelDistanceM = inputs.wheelAngalRad * (WHEEL_DIAMETER_METERS/2.0);

        inputs.rightAngleRad = righEncoder.getPosition();
        inputs.rightAngularVelocityRadPerSec = righEncoder.getVelocity();
        inputs.rightAppliedPower = rightMotor.getAppliedOutput();
        inputs.rightCurrentDrawAmps = rightMotor.getOutputCurrent();
    }

    @Override
    public void setSpeeds(double leftPower, double rightPower) {
        leftMotor.set(leftPower);
        rightMotor.set(rightPower);
    }
}
