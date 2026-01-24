package frc.robot.CatzAbstractions.Bases;

import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.Utilities.Setpoint;
import frc.robot.Utilities.EqualsUtil;

import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class ServoMotorSubsystem<S extends GenericMotorIO<I>, I extends GenericMotorIO.MotorIOInputs> extends GenericMotorSubsystem<S, I>  {

	protected final String name;
	protected final Angle epsilonThreshold;

	private double manualSpeed = 0.0;
	protected boolean isFullManual = false;

	public ServoMotorSubsystem(S io, I inputs, String name, Angle epsilonThreshold) {
		super(io, inputs, name);
		this.name = name;
		this.epsilonThreshold = epsilonThreshold;
	}

	@Override
	public void periodic() {
		super.periodic();

		if (isFullManual) {
			runFullManual(manualSpeed);
		}
	}

	public void runFullManual(double speed) {
		if (Math.abs(speed) < 0.1) {
			io.setMotionMagicSetpoint(getPosition());
		} else {
			io.setDutyCycleSetpoint(speed);
		}
	}

	public void useSoftLimits(boolean enable) {
		io.useSoftLimits(enable);
	}

	/**
	 * Determines whether the subsystem is near it's position setpoint.
	 *
	 * @return True if currently near setpoint, false if not. Returns false if not
	 *         in position control.
	 */
	public boolean nearPositionSetpoint() {
		return (setpoint.mode.isPositionControl()) && nearPosition(Units.Rotations.of(getPosition()));
	}

	/**
	 * Creates a Command that goes to a setpoint and then waits until the mechanism is the setpoint's position.
	 *
	 * @param mechanismPosition Position to evaluate proximity to.
	 * @return A new Command to apply setpoint and wait.
	 */
	public Command setpointCommandWithWait(Setpoint setpoint) {
		return waitForPositionCommand(Units.Rotations.of(setpoint.baseUnits))
				.deadlineFor(followSetpointCommand(()->setpoint));
	}

	/**
	 * Creates a Command that waits until the mechanism is near a given position.
	 *
	 * @param mechanismPosition Position to evaluate proximity to.
	 * @return A wait command.
	 */
	public Command waitForPositionCommand(Angle mechanismPosition) {
		return Commands.waitUntil(() -> {
			return nearPosition(mechanismPosition);
		});
	}

	/**
	 * Determines whether the subsystem is near a given position.
	 *
	 * @param mechanismPosition Position to compare to.
	 * @return True if near provided position, false if not.
	 */
	public boolean nearPosition(Angle mechanismPosition) {
		return EqualsUtil.epsilonEquals(
				getPosition(),
				mechanismPosition.in(Units.Rotations),
				epsilonThreshold.in(Units.Rotations));
	}

	public void setCurrentPosition(Angle position) {
		io.setCurrentPosition(position.in(Units.Rotations));
	}

	public Command setCurrentPositionCommand(Angle position){
		return runOnce(() -> setCurrentPosition(position));
	}

	public Command fullManualCommand(Supplier<Double> speed) {
		return runOnce(() -> {
			isFullManual = true;
			manualSpeed = speed.get();
			applySetpoint(Setpoint.withBrakeSetpoint());
		}).until(() -> !isFullManual).andThen(runOnce(() -> {
			isFullManual = false;
			manualSpeed = 0.0;
		}));
	}
}
