package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.utility.KalmanFilterReplacment;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.Constants.*;

import org.littletonrobotics.junction.Logger;

public class SwerveModule {
        private final String name;
        private final SwerveModuleIO io;
        private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

        private final Translation2d positionVector;
        private Matrix<N3, N1> reference;
        private SwerveModuleState targeState = new SwerveModuleState(0, new Rotation2d(0));

        private final PIDController steerPID;
        private final PIDController drivePID;

        private final double steerkP = 1.0;
        private final double steerkI = 0.0;
        private final double steerkD = 0.0;
    
        private final double drivekP = 0.7;
        private final double drivekI = 0.0;
        private final double drivekD = 0.1;

        public SwerveModule(String name, SwerveModuleIO io, Translation2d positionVector) {
                this.name = name;
                this.io = io;
                this.positionVector = positionVector;

                steerPID = new PIDController(steerkP, steerkI, steerkD);
                drivePID = new PIDController(drivekP, drivekI, drivekD);
        }

        public void updateInputs() {
                io.updateInputs(inputs);
                Logger.getInstance().processInputs("Drivetrain/" + name, inputs);

                steerPID.setSetpoint(targeState.angle.getRadians());
                drivePID.setSetpoint(targeState.speedMetersPerSecond);
        
                double steerPower = steerPID.calculate(inputs.absoluteAngleRad);
                double drivePower = drivePID.calculate(inputs.wheelAngularVelocityRadPerSec);
        
                double topMotor = (steerPower + drivePower);
                double bottomMotor = - (steerPower - drivePower);

                setPowers(bottomMotor,topMotor);
        }

        public void setTargetState(SwerveModuleState state) {
                this.targeState = state;
                setReference(VecBuilder.fill(state.angle.getRadians(), 0,
                                -state.speedMetersPerSecond / (WHEEL_DIAMETER_METERS / 2.0)));
        }

        public void setReference(Matrix<N3, N1> reference) {
                this.reference = reference;
        }
 
        public SwerveModulePosition getCurrentPosition() {
                return new SwerveModulePosition(
                                inputs.wheelDistanceM,
                                new Rotation2d(inputs.absoluteAngleRad));
        }

        public SwerveModuleState getSwerveModuleState() {
                return new SwerveModuleState(inputs.wheelSpeedMPerSec, new Rotation2d(inputs.absoluteAngleRad));
        }

        private void setPowers(double bottomPower, double topPower) {
                io.setVoltages(bottomPower, topPower);
                
        }

        // this is the location of the module with respect to the robot.
        public Translation2d getModuleLocation() {
                return positionVector;
        }
}
  