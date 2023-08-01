// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;

/** Add your docs here. */
public class KalmanFilterReplacment<States extends Num, Inputs extends Num, Outputs extends Num>
    extends KalmanFilter<States, Inputs, Outputs> {

  private final Nat<States> m_states;

  private final LinearSystem<States, Inputs, Outputs> m_plant;

  /** The steady-state Kalman gain matrix. */
  private final Matrix<States, Outputs> m_K;

  /** The state estimate. */
  private Matrix<States, N1> m_xHat;

  public KalmanFilterReplacment(
      Nat<States> states,
      Nat<Outputs> outputs,
      LinearSystem<States, Inputs, Outputs> plant,
      Matrix<States, N1> stateStdDevs,
      Matrix<Outputs, N1> measurementStdDevs,
      double dtSeconds) {
    super(states, outputs, plant, stateStdDevs, measurementStdDevs, dtSeconds);

    this.m_states = states;

    this.m_plant = plant;

    m_xHat = new Matrix<>(m_states, Nat.N1());

    m_K = new Matrix<>(m_states, outputs);
  }

  @Override
  public Matrix<States, Outputs> getK() {
    return m_K;
  }

  /**
   * Returns an element of the steady-state Kalman gain matrix K.
   *
   * @param row Row of K.
   * @param col Column of K.
   * @return the element (i, j) of the steady-state Kalman gain matrix K.
   */
  @Override
  public double getK(int row, int col) {
    return m_K.get(row, col);
  }

  /**
   * Set initial state estimate x-hat.
   *
   * @param xhat The state estimate x-hat.
   */
  @Override
  public void setXhat(Matrix<States, N1> xhat) {
    this.m_xHat = xhat;
  }

  /**
   * Set an element of the initialz state estimate x-hat.
   *
   * @param row   Row of x-hat.
   * @param value Value for element of x-hat.
   */
  @Override
  public void setXhat(int row, double value) {
    m_xHat.set(row, 0, value);
  }

  /**
   * Returns the state estimate x-hat.
   *
   * @return The state estimate x-hat.
   */
  @Override
  public Matrix<States, N1> getXhat() {
    return m_xHat;
  }

  /**
   * Returns an element of the state estimate x-hat.
   *
   * @param row Row of x-hat.
   * @return the state estimate x-hat at that row.
   */
  public double getXhat(int row) {
    return m_xHat.get(row, 0);
  }

  /**
   * Correct the state estimate x-hat using the measurements in y.
   *
   * @param u Same control input used in the last predict step.
   * @param y Measurement vector.
   */
  public void correct(Matrix<Inputs, N1> u, Matrix<Outputs, N1> y) {
    /**
        final var C = m_plant.getC();
    final var D = m_plant.getD();
    // x̂ₖ₊₁⁺ = x̂ₖ₊₁⁻ + K(y − (Cx̂ₖ₊₁⁻ + Duₖ₊₁))
    m_xHat = m_xHat.plus(m_K.times(y.minus(C.times(m_xHat).plus(D.times(u)))));
    */

    m_xHat = (Matrix<States, N1>) y;
  }
}
