package frc.robot.CatzAbstractions.io;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Utilities.Setpoint;



public interface GenericMotorIO {


  @AutoLog
  public static class MotorIOInputs {

    public boolean isLeaderConnected = false;
    public boolean[] isFollowerConnected = new boolean[] {};

    public double position = 0.0;
    public double velocityRPS = 0.0;
    public double accelerationRPS = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};

  }

  public default void updateInputs(MotorIOInputs inputs) {}

  public default void setCurrentPosition(double mechanismPosition) {}

  /**
   * Set's the mechanism's current location as zero.
   */
  public default void zeroSensors() {}

  /**
   * Sets the motor to brake or coast.
   *
   * @param wantsBrake Whether to brake or coast. True is brake, false is coast.
   */
  public default void setNeutralBrake(boolean wantsBrake) {}

  /**
   * Sets whether to enable or disble soft limits.
   *
   * @param enable Whether to enable or disbale soft limits. True is enable, false is disable.
   */
  public default void useSoftLimits(boolean enable) {}

  /**
   * Sets the motor to be idle. Should not be called directly, only applied through Setpoints.
   */
  public default void setNeutralSetpoint() {}

  /**
   * Sets the motor to be coasting. Should not be called directly, only applied through Setpoints.
   */
  public default void setCoastSetpoint() {}

  /**
   * Sets the motor to run at a given voltage. Should not be called directly, only applied through Setpoints.
   *
   * @param voltage Voltage to run at (volts).
   */
  public default void setVoltageSetpoint(double voltage) {}

  /**
   * Sets the motor to use motion magic control to go to a given position. Should not be called directly, only applied through Setpoints.
   *
   * @param mechanismPosition Mechanism position to go to in rotations.
   */
  public default void setMotionMagicSetpoint(double mechanismPosition) {}

  /**
   * Sets the motor to go to a given velocity. Should not be called directly, only applied through Setpoints.
   *
   * @param mechanismVelocity Mechanism velocity to go to (rad/s or RPM depending on implementation).
   */
  public default void setVelocitySetpoint(double mechanismVelocity) {}

  /**
   * Sets the motor to run at a percentage of its max voltage. Should not be called directly, only applied through Setpoints.
   *
   * @param percent Percentage of max voltage to run at (0.0 to 1.0).
   */
  public default void setDutyCycleSetpoint(double percent) {}

  /**
   * Sets the motor to use PID control to go to a given position. Should not be called directly, only applied through Setpoints.
   *
   * @param mechanismPosition Mechanism position to go to (radians or degrees depending on implementation).
   */
  public default void setPositionSetpoint(double mechanismPosition) {}

  /**
   * Enables this MotorIO. Immediatly applies the last set Setpoint including Setpoints set when disabled. MotorIO is enabled by default.
   */
  public default void enable() {}

  public default void setGainsSlot0(double p, double i, double d, double s, double v, double a, double g) {}

  public default void setGainsSlot1(double p, double i, double d, double s, double v, double a, double g) {}

  public default void setMotionMagicParameters(double velocity, double acceleration, double jerk) {}

  /**
   * Disabled this MotorIO. Setpoints can still be set when disabled but will not be applied until re-enabled.
   */
  public default void disable() {}

  public default void stop() {}

  public default void setNeutralMode(TalonFX fx, NeutralModeValue neutralMode) {}

  public default void setNeutralMode(TalonFXS fx, NeutralModeValue neutralMode) {}
}
