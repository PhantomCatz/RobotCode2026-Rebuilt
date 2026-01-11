package frc.robot.CatzAbstractions.Bases;

import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.Utilities.EqualsUtil;
import frc.robot.Utilities.Setpoint;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class PivotMotorSubsystem extends GenericMotorSubsystem {

	//TODO make this a child of ServoMotorSubsystem

	protected final double epsilonThreshold;
	private Setpoint setpoint = Setpoint.withBrakeSetpoint();
	private double manualSpeed = 0.0;
	private boolean isFullManual = false;


	public PivotMotorSubsystem(GenericMotorIO io, String name, double epsilonThreshold) {
		super(io, name);
		this.epsilonThreshold = epsilonThreshold;
	}


	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs(name, inputs);

		if(DriverStation.isDisabled() || setpoint == null) {
			// Disabled
			io.stop();
		} else if (isFullManual) {
			runFullManual(manualSpeed);
		} else if (setpoint != null) {
			setpoint.apply(io);
		} else {
			// No action
			io.stop();
		}
	}

	public void runFullManual(double speed) {
		
		if(Math.abs(speed) < 0.1) {
			io.setMotionMagicSetpoint(getPosition());
		}else{
			io.setDutyCycleSetpoint(speed);
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
		return EqualsUtil.epsilonEquals(
				inputs.position,
				mechanismPosition,
				epsilonThreshold);
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
