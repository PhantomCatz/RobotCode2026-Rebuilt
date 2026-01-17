package frc.robot.Utilities;

import java.util.function.UnaryOperator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.CatzAbstractions.io.GenericMotorIO;

public class Setpoint {
    private final UnaryOperator<GenericMotorIO> applier;

    public final Mode mode;
    public final double baseUnits;

    /**
     * Creates a setpoint with a given applier, control mode, and base units
     * equivalent.
     *
     * @param applier   What to apply to ServoMotorIO when the setpoint is set.
     * @param mode      Control mode to register for this setpoint.
     * @param baseUnits Setpoint's target in it's base form of units as a double.
     */
    private Setpoint(UnaryOperator<GenericMotorIO> applier, Mode mode, double baseUnits) {
        this.applier = applier;
        this.mode = mode;
        this.baseUnits = baseUnits;
    }

    /**
     * Creates a setpoint with a completely custom applier, control mode, and base
     * units.
     *
     * @param applier   What to apply to ServoMotorIO when the setpoint is set.
     * @param mode      Control mode to register for this setpoint.
     * @param baseUnits Setpoint's target in it's base form of units as a double.
     */
    public static Setpoint withCustomSetpoint(UnaryOperator<GenericMotorIO> applier, Mode mode, double baseUnits) {
        return new Setpoint(applier, mode, baseUnits);
    }

    /**
		 * Creates a setpoint to use motion magic control to go to a position.
		 *
		 * @param motionMagicSetpoint Posiiton to go to in mechanism units.
		 * @return A new Setpoint.
		 */
		public static Setpoint withMotionMagicSetpoint(Angle motionMagicSetpoint) {
			UnaryOperator<GenericMotorIO> applier = (GenericMotorIO io) -> {
				io.setMotionMagicSetpoint(motionMagicSetpoint.in(Units.Rotations));
				return io;
			};
			return new Setpoint(applier, Mode.MOTIONMAGIC, motionMagicSetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to use PID control to go to a position.
		 *
		 * @param positionSetpoint Posiiton to go to in mechanism units.
		 * @return A new Setpoint.
		 */
		public static Setpoint withPositionSetpoint(Angle positionSetpoint) {
			UnaryOperator<GenericMotorIO> applier = (GenericMotorIO io) -> {
				io.setPositionSetpoint(positionSetpoint.in(Units.Rotations));
				return io;
			};
			return new Setpoint(applier, Mode.POSITIONPID, positionSetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to go to a velocity.
		 *
		 * @param velocitySetpoint Velocity to go to in mechanism units.
		 * @return A new Setpoint.
		 */
		public static Setpoint withVelocitySetpoint(AngularVelocity velocitySetpoint) {
			UnaryOperator<GenericMotorIO> applier = (GenericMotorIO io) -> {
				io.setVelocitySetpoint(velocitySetpoint.in(Units.RotationsPerSecond));
				return io;
			};
			return new Setpoint(applier, Mode.VELOCITY, velocitySetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to run at a voltage.
		 *
		 * @param voltage Voltage to run at.
		 * @return A new Setpoint.
		 */
		public static Setpoint withVoltageSetpoint(Voltage voltage) {
			UnaryOperator<GenericMotorIO> applier = (GenericMotorIO io) -> {
				io.setVoltageSetpoint(voltage.in(Units.Volts));
				return io;
			};
			return new Setpoint(applier, Mode.VOLTAGE, voltage.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to run at a percent of maximum voltage.
		 *
		 * @param percent Percent to run at.
		 * @return A new Setpoint.
		 */
		public static Setpoint withDutyCycleSetpoint(Dimensionless percent) {
			UnaryOperator<GenericMotorIO> applier = (GenericMotorIO io) -> {
				io.setDutyCycleSetpoint(percent.in(Units.Percent));
				return io;
			};
			return new Setpoint(applier, Mode.DUTY_CYCLE, percent.baseUnitMagnitude());
		}

    /**
     * Creates a setpoint to use motion magic control to go to a position.
     *
     * @param motionMagicSetpoint Posiiton to go to in mechanism units.
     * @return A new Setpoint.
     */
    public static Setpoint withMotionMagicSetpoint(double motionMagicSetpoint) {
        UnaryOperator<GenericMotorIO> applier = (GenericMotorIO io) -> {
            io.setMotionMagicSetpoint(motionMagicSetpoint);
            return io;
        };
        return new Setpoint(applier, Mode.MOTIONMAGIC, motionMagicSetpoint);
    }


    /**
     * Creates a setpoint to use PID control to go to a position.
     *
     * @param positionSetpoint Position to go to in mechanism units.
     * @return A new Setpoint.
     */
    public static Setpoint withPositionSetpoint(double positionSetpoint) {
        UnaryOperator<GenericMotorIO> applier = (GenericMotorIO io) -> {
            io.setPositionSetpoint(positionSetpoint);
            return io;
        };
        return new Setpoint(applier, Mode.POSITIONPID, positionSetpoint);
    }

    /**
     * Creates a setpoint to go to a velocity.
     *
     * @param velocitySetpoint Velocity to go to in mechanism units.
     * @return A new Setpoint.
     */
    public static Setpoint withVelocitySetpoint(double velocitySetpoint) {
        UnaryOperator<GenericMotorIO> applier = (GenericMotorIO io) -> {
            io.setVelocitySetpoint(velocitySetpoint);
            return io;
        };
        return new Setpoint(applier, Mode.VELOCITY, velocitySetpoint);
    }

    public static Setpoint withVelocitySetpointVoltage(double velocitySetpoint) {
        UnaryOperator<GenericMotorIO> applier = (GenericMotorIO io) -> {
            io.setVelocitySetpointVoltage(velocitySetpoint);
            return io;
        };
        return new Setpoint(applier, Mode.VELOCITY, velocitySetpoint);
    }

    /**
     * Creates a setpoint to run at a voltage.
     *
     * @param voltage Voltage to run at.
     * @return A new Setpoint.
     */
    public static Setpoint withVoltageSetpoint(double voltage) {
        UnaryOperator<GenericMotorIO> applier = (GenericMotorIO io) -> {
            io.setVoltageSetpoint(voltage);
            return io;
        };
        return new Setpoint(applier, Mode.VOLTAGE, voltage);
    }

    /**
     * Creates a setpoint to run at a percent of maximum voltage.
     *
     * @param percent Percent to run at.
     * @return A new Setpoint.
     */
    public static Setpoint withDutyCycleSetpoint(double percent) {
        UnaryOperator<GenericMotorIO> applier = (GenericMotorIO io) -> {
            io.setDutyCycleSetpoint(percent);
            return io;
        };
        return new Setpoint(applier, Mode.DUTY_CYCLE, percent);
    }

    /**
     * Creates a setpoint to idle.
     *
     * @return A new Setpoint.
     */
    public static Setpoint withBrakeSetpoint() {
        UnaryOperator<GenericMotorIO> applier = (GenericMotorIO io) -> {
            io.setNeutralBrake(true);
            return io;
        };
        return new Setpoint(applier, Mode.IDLE, 0.0);
    }

    /**
     * Creates a setpoint to coast.
     *
     * @return A new Setpoint.
     */
    public static Setpoint withCoastSetpoint() {
        UnaryOperator<GenericMotorIO> applier = (GenericMotorIO io) -> {
            io.setNeutralBrake(false);
            return io;
        };
        return new Setpoint(applier, Mode.IDLE, 0.0);
    }

    public void apply(GenericMotorIO io) {
        applier.apply(io);
    }

    public enum Mode {
		IDLE,
		VOLTAGE,
		MOTIONMAGIC,
		VELOCITY,
		DUTY_CYCLE,
		POSITIONPID;

		/**
		 * Gets whether the control mode is based on position. Motion Magic and Position PID control count as position.
		 *
		 * @return True if in position control, false if not.
		 */
		public boolean isPositionControl() {
			return switch (this) {
				case MOTIONMAGIC, POSITIONPID -> true;
				default -> false;
			};
		}

		/**
		 * Gets whether the control mode is based on velocity.
		 *
		 * @return True if in velocity control, false if not.
		 */
		public boolean isVelocityControl() {
			return switch (this) {
				case VELOCITY -> true;
				default -> false;
			};
		}

		/**
		 * Gets whether the control mode is neutral. Only Idle counts as neutral
		 *
		 * @return True if in velocity control, false if not.
		 */
		public boolean isNeutralControl() {
			return switch (this) {
				case IDLE -> true;
				default -> false;
			};
		}

		/**
		 * Gets whether the control mode is based on voltage. Voltage and Duty Cycle control count as voltage.
		 *
		 * @return True if in voltage control, false if not.
		 */
		public boolean isVoltageControl() {
			return switch (this) {
				case VOLTAGE, DUTY_CYCLE -> true;
				default -> false;
			};
		}
	}
}
