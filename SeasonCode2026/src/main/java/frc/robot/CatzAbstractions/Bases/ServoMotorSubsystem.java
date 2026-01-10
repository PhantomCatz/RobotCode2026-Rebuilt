package frc.robot.CatzAbstractions.Bases;

import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.Utilities.EpsilonEquals;
import frc.robot.Utilities.Setpoint;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class ServoMotorSubsystem extends GenericMotorSubsystem {

	protected final GenericMotorIO io;
	protected final String name;
	protected final double epsilonThreshold;
	private Setpoint setpoint = Setpoint.withBrakeSetpoint();
	private double manualSpeed = 0.0;
	private final double slammingThreshold;
	private boolean isFullManual = false;


	public ServoMotorSubsystem(GenericMotorIO io, String name, double epsilonThreshold, double slammingThreshold) {
		super(io, name);
		this.io = io;
		this.name = name;
		this.epsilonThreshold = epsilonThreshold;
		this.slammingThreshold = slammingThreshold;
	}


	@Override
	public void periodic() {
		super.periodic();

		if(DriverStation.isDisabled() || setpoint == null) {
			// Disabled
			io.stop();
		} else if (isFullManual) {
			runFullManual(manualSpeed);
		} else if(setpoint.baseUnits <= slammingThreshold) {  // Prevent slamming if our setpoint and current position is very low
			if(getPosition() <= slammingThreshold) {
				io.stop();
			} else {
				setpoint.apply(io);

			}
		}else if (setpoint.baseUnits > slammingThreshold) {
			setpoint.apply(io);
		} else {
			// No action
			io.stop();
		}
	}

	public void runFullManual(double speed) {
		io.setDutyCycleSetpoint(speed);

		if(Math.abs(speed) < 0.1) {
			io.setMotionMagicSetpoint(getPosition());
		}

	}

	public void useSoftLimits(boolean enable) {
		io.useSoftLimits(enable);
	}

	/**
	 * Determines whether the subsystem is near it's position setpoint.
	 *
	 * @return True if currently near setpoint, false if not. Returns false if not in position control.
	 */
	public boolean nearPositionSetpoint() {
		return nearPosition(inputs.position);
	}

	/**
	 * Determines whether the subsystem is near a given position.
	 *
	 * @param mechanismPosition Position to compare to.
	 * @return True if near provided position, false if not.
	 */
	public boolean nearPosition(double mechanismPosition) {
		return EpsilonEquals.epsilonEquals(
				inputs.position,
				mechanismPosition,
				epsilonThreshold);
	}

	public double getSetpointDoubleInUnits() {
		return setpoint.baseUnits;
	}

	public void setCurrentPosition(double position) {
		io.setCurrentPosition(position);
	}

	public void applySetpoint(Setpoint setpoint) {
		this.setpoint = setpoint;
	}

	/**
	 * Creates a one time, instantaneus command for the subsystem to go to a given Setpoint.
	 *
	 * @param setpoint Setpoint to go to.
	 * @return One time Command for the subsystem.
	 */
	public Command setpointCommand(Setpoint setpoint) {
		return runOnce(() -> applySetpoint(setpoint));
	}

	/**
	 * Creates a continous command for the subsystem to repeatedly go to a supplied setpoint.
	 *
	 * @param setpoint Supplier of setpoint to go to.
	 * @return Continuous Command for the subsystem.
	 */
	public Command followSetpointCommand(Supplier<Setpoint> supplier) {
		return run(() -> applySetpoint(supplier.get()));
	}

	public Command fullManualCommand(Supplier<Double> speed) {
		return runOnce(() -> {
			isFullManual = true;
			manualSpeed = speed.get();
		}).until(() -> !isFullManual).andThen(runOnce(() -> {
			isFullManual = false;
			manualSpeed = 0.0;
		}));
	}





}
