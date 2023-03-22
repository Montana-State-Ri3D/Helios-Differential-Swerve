package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class SwerveModuleIOJuno implements SwerveModuleIO {
    private static final double WHEEL_NOMINAL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    // This value accounts for the wearing down of the tread over the course of a
    // competition, as well as the "squish factor" between
    // the wheels and the carpet. Function may be added down the line to dynamically
    // adjust this value in the field.
    private static final double TREADWEAR = Units.inchesToMeters(0.125);
    private static final double WHEEL_DIAMETER_METERS = WHEEL_NOMINAL_DIAMETER_METERS - TREADWEAR;

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final CANCoder steerEncoder;
    private final RelativeEncoder lefEncoder;
    private final RelativeEncoder righEncoder;

    private final PIDController steerPID;
    private final PIDController drivePID;

    private final double steerkP = 0.0;
    private final double steerkI = 0.0;
    private final double steerkD = 0.0;

    private final double drivekP = 0.0;
    private final double drivekI = 0.0;
    private final double drivekD = 0.0;

    private final double driveRadio = 0.0;
    private final double steerRadio = 0.0;

    private final double offset;


    public SwerveModuleIOJuno(
            int leftMotorID,
            int rightMotorID,
            int steerEncoderID,
            double offset) {
        this.offset = offset;

        leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
        steerEncoder = new CANCoder(steerEncoderID);
        this.lefEncoder = leftMotor.getEncoder();
        this.righEncoder = rightMotor.getEncoder();

        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);

        steerPID = new PIDController(steerkP, steerkI, steerkD);
        drivePID = new PIDController(drivekP, drivekI, drivekD);

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

        inputs.drivePositionMeters = (lefEncoder.getPosition() + righEncoder.getPosition()) / 2.0;
        inputs.absoluteAngleRad = steerEncoder.getPosition();
        inputs.absoluteAngularVelocityRadPerSec = steerEncoder.getVelocity();

        inputs.rightAngleRad = righEncoder.getPosition();
        inputs.rightAngularVelocityRadPerSec = righEncoder.getVelocity();
        inputs.rightAppliedPower = rightMotor.getAppliedOutput();
        inputs.rightCurrentDrawAmps = rightMotor.getOutputCurrent();
    }

    @Override
    public void drive(double velocityMetersPerSec, double targetSteerAngle) {
        steerPID.setSetpoint(targetSteerAngle);
        drivePID.setSetpoint(velocityMetersPerSec);

        double steerPower = steerPID.calculate(steerEncoder.getPosition());
        double drivePower = drivePID.calculate((lefEncoder.getVelocity() + righEncoder.getVelocity()) / 2.0);

        rightMotor.set(steerPower + drivePower);
        leftMotor.set(steerPower - drivePower);
    }
}
