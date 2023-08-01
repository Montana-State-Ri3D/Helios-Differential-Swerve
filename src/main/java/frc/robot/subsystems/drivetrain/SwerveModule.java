package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
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

import static frc.robot.Constants.*;

import org.littletonrobotics.junction.Logger;

public class SwerveModule {
        private final String name;
        private final SwerveModuleIO io;
        private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

        private final Translation2d positionVector;
        private final LinearSystemLoop<N3, N2, N3> swerveControlLoop;
        private Matrix<N3, N1> reference; // same thing as a set point.
        private Matrix<N2, N1> u;

        public SwerveModule(String name, SwerveModuleIO io, Translation2d positionVector) {
                this.name = name;
                this.io = io;

                reference = Matrix.mat(Nat.N3(), Nat.N1()).fill(0, 0, 0);
                this.positionVector = positionVector;

                // Creates a Linear System of our Differential Swerve Module.
                LinearSystem<N3, N2, N3> swerveModuleModel = createDifferentialSwerveModule(DCMotor.getNEO(2),
                                INERTIA_STEER,
                                INERTIA_WHEEL,
                                STEER_RADIO,
                                DRIVE_RADIO);

                // Creates a Kalman Filter as our Observer for our module. Works
                // since system is linear.
                KalmanFilterReplacment<N3, N2, N3> swerveObserver = new KalmanFilterReplacment<>(Nat.N3(), Nat.N3(), swerveModuleModel,
                                Matrix.mat(Nat.N3(), Nat.N1()).fill(
                                                MODEL_AZIMUTH_ANGLE_NOISE,
                                                MODEL_AZIMUTH_ANG_VELOCITY_NOISE,
                                                MODEL_WHEEL_ANG_VELOCITY_NOISE),
                                Matrix.mat(Nat.N3(), Nat.N1()).fill(
                                                SENSOR_AZIMUTH_ANGLE_NOISE,
                                                SENSOR_AZIMUTH_ANG_VELOCITY_NOISE,
                                                SENSOR_WHEEL_ANG_VELOCITY_NOISE),
                                kDt);
                // Creates an LQR controller for our Swerve Module.
                LinearQuadraticRegulator<N3, N2, N3> swerveController = new LinearQuadraticRegulator<>(
                                swerveModuleModel,
                                // Q Vector/Matrix Maximum error tolerance
                                VecBuilder.fill(Q_AZIMUTH,
                                                Q_AZIMUTH_ANG_VELOCITY,
                                                Q_WHEEL_ANG_VELOCITY),
                                // R Vector/Matrix Maximum control effort.
                                VecBuilder.fill(CONTROL_EFFORT,
                                                CONTROL_EFFORT),
                                kDt);

                // Creates a LinearSystemLoop that contains the Model,
                // Controller, Observer, Max Volts,
                // Update Rate.
                swerveControlLoop = new LinearSystemLoop<>(swerveModuleModel, swerveController, swerveObserver, VOLTAGE,
                                kDt);

                // Initializes the vectors and matrices.
                swerveControlLoop.reset(VecBuilder.fill(0, 0, 0));
                u = VecBuilder.fill(0, 0);
        }

        public void updateInputs() {
                io.updateInputs(inputs);
                Logger.getInstance().processInputs("Drivetrain/" + name, inputs);

                // sets the next reference / setpoint.
                swerveControlLoop.setNextR(reference);
                // updates the kalman filter with new data points.
                swerveControlLoop.correct(VecBuilder.fill(-inputs.absoluteAngleRadContiuious,inputs.absoluteAngularVelocityRadPerSec, inputs.wheelAngularVelocityRadPerSec));
                // predict step of kalman filter.
                predict();

                KalmanFilter<N3, N2, N3> swerveObserver = swerveControlLoop.getObserver();

                double[] kalmanout = new double[] {-swerveObserver.getXhat(0),swerveObserver.getXhat(1),swerveObserver.getXhat(2)};

                Logger.getInstance().recordOutput("Drivetrain/" + name + "/Kalman",kalmanout );

                setPowers(getBottomNextVoltage(), getTopNextVoltage());
        }

        public void setTargetState(SwerveModuleState state) {
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

        // use custom predict() function for as absolute encoder azimuth angle
        // and the angular velocity
        // of the module need to be continuous.
        private void predict() {
                // creates our input of voltage to our motors of u = K(r-x) but
                // need to wrap angle to be
                // continuous
                u = swerveControlLoop.clampInput(swerveControlLoop.getController().getK().times(// profiledReference())
                                wrapAngle(swerveControlLoop.getNextR(), swerveControlLoop.getXHat(), -Math.PI,
                                                Math.PI))
                                .plus(VecBuilder.fill(
                                                FEED_FORWARD * reference.get(2, 0),
                                                -FEED_FORWARD * reference.get(2, 0))));
                swerveControlLoop.getObserver().predict(u, kDt);
        }

        private void setPowers(double bottomPower, double topPower) {
                io.setVoltages(-bottomPower, topPower);
        }

        /**
         * gets the wanted voltage from our control law. u = K(r-x) our control
         * law is slightly different as we need to be continuous. Check method
         * predict() for calculations.
         *
         * @return bottom wanted voltage
         */
        private double getBottomNextVoltage() {
                return u.get(0, 0);
        }

        private double getTopNextVoltage() {
                return u.get(1, 0);
        }

        // this is the location of the module with respect to the robot.
        public Translation2d getModuleLocation() {
                return positionVector;
        }

        /**
         * wraps angle so that absolute encoder can be continues. (i.e) No
         * issues when switching between -PI and PI as they are the same point
         * but different values.
         *
         * @param reference is the Matrix that contains the reference wanted
         *                  such as [Math.PI, 0, 100].
         * @param xHat      is the predicted states of our system. [Azimuth Angle,
         *                  Azimuth Angular Velocity, Wheel Angular Velocity].
         * @param minAngle  is the minimum angle in our case -PI.
         * @param maxAngle  is the maximum angle in our case PI.
         */
        private Matrix<N3, N1> wrapAngle(Matrix<N3, N1> reference, Matrix<N3, N1> xHat, double minAngle, double maxAngle) {
                double angleError = reference.get(0, 0) - inputs.absoluteAngleRadContiuious;
                double positionError = MathUtil.inputModulus(angleError, minAngle, maxAngle);
                Matrix<N3, N1> error = reference.minus(xHat);
                return VecBuilder.fill(positionError, error.get(1, 0), error.get(2, 0));
        }

        /**
         * Creates a StateSpace model of a differential swerve module.
         *
         * @param motor is the motor used.
         * @param Js    is the Moment of Inertia of the steer component.
         * @param Jw    is the Moment of Inertia of the wheel component.
         * @param Gs    is the Gear Ratio of the steer.
         * @param Gw    is the Gear Ratio of the wheel.
         * @return LinearSystem of state space model.
         */
        private static LinearSystem<N3, N2, N3> createDifferentialSwerveModule(DCMotor motor, double Js, double Jw,
                        double Gs, double Gw) {
                var Cs = -((Gs * motor.KtNMPerAmp) / (motor.KvRadPerSecPerVolt * motor.rOhms * Js));
                var Cw = -((Gw * motor.KtNMPerAmp) / (motor.KvRadPerSecPerVolt * motor.rOhms * Jw));
                var Vs = 0.5 * ((Gs * motor.KtNMPerAmp) / (motor.rOhms * Js));
                var Vw = 0.5 * ((Gw * motor.KtNMPerAmp) / (motor.rOhms * Jw));

                var A = Matrix.mat(Nat.N3(), Nat.N3()).fill(0.0, 1.0, 0.0, 0.0, Gs * Cs, 0.0, 0.0, 0.0, Gw * Cw);
                var B = Matrix.mat(Nat.N3(), Nat.N2()).fill(0.0, 0.0, Vs, Vs, Vw, -Vw);
                var C = Matrix.mat(Nat.N3(), Nat.N3()).fill(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
                var D = Matrix.mat(Nat.N3(), Nat.N2()).fill(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                return new LinearSystem<>(A, B, C, D);
        }
}
  