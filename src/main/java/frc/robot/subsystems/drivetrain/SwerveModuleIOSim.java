package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

import static frc.robot.Constants.*;

public class SwerveModuleIOSim implements SwerveModuleIO {
    // This value accounts for the wearing down of the tread over the course of a
    // competition, as well as the "squish factor" between
    // the wheels and the carpet. Function may be added down the line to dynamically
    // adjust this value in the field.
    private static final double WHEEL_DIAMETER_METERS = WHEEL_NOMINAL_DIAMETER_METERS - TREADWEAR;

    private final EncoderSim steerEncoder;

    private final PIDController steerPID;
    private final PIDController drivePID;

    private final double steerkP = 0.10;
    private final double steerkI = 0.0;
    private final double steerkD = 0.0;

    private final double drivekP = 0.00001;
    private final double drivekI = 0.0;
    private final double drivekD = 0.0;

    private final DCMotor leftMotor;
    private final DCMotor rightMotor;

    private double leftPower;
    private double rightPower;

    private final DCMotorSim leftMotorSim;
    private final DCMotorSim rightMotorSim;


    private final double momentjKgMetersSquared = 1;


    public SwerveModuleIOSim() {

        leftMotor = DCMotor.getNEO(1);
        rightMotor = DCMotor.getNEO(1);
        leftMotorSim = new DCMotorSim(leftMotor,DRIVE_RADIO, momentjKgMetersSquared);
        rightMotorSim = new DCMotorSim(rightMotor,DRIVE_RADIO, momentjKgMetersSquared);

        steerEncoder = EncoderSim.createForIndex(1);

        steerPID = new PIDController(steerkP, steerkI, steerkD);
        drivePID = new PIDController(drivekP, drivekI, drivekD);
        leftPower = 0;
        rightPower = 0;
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.leftAngleRad = leftMotorSim.getAngularPositionRad();
        inputs.leftAngularVelocityRadPerSec = leftMotorSim.getAngularVelocityRadPerSec();
        inputs.leftAppliedPower = leftPower;
        inputs.leftCurrentDrawAmps = leftMotorSim.getCurrentDrawAmps();

        inputs.drivePositionMeters = (leftMotorSim.getAngularPositionRad() + rightMotorSim.getAngularPositionRad()) / 2.0;
        inputs.absoluteAngleRad = Math.toRadians(steerEncoder.getDistance());
        inputs.absoluteAngularVelocityRadPerSec = Math.toRadians(steerEncoder.getRate());

        inputs.rightAngleRad = rightMotorSim.getAngularPositionRad();
        inputs.rightAngularVelocityRadPerSec = rightMotorSim.getAngularVelocityRadPerSec();
        inputs.rightAppliedPower = rightPower;
        inputs.rightCurrentDrawAmps = rightMotorSim.getCurrentDrawAmps();
    }

    @Override
    public void drive(double velocityMetersPerSec, double targetSteerAngle) {
        steerPID.setSetpoint(targetSteerAngle);
        drivePID.setSetpoint(velocityMetersPerSec);

        double steerPower = steerPID.calculate(steerEncoder.getDistance());
        double drivePower = drivePID.calculate((leftMotorSim.getAngularVelocityRadPerSec() + rightMotorSim.getAngularVelocityRadPerSec()) / 2.0);

        rightMotorSim.setInput(steerPower + drivePower);
        leftMotorSim.setInput(steerPower - drivePower);
    }
}
