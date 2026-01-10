package frc.robot.CatzAbstractions.Bases;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.CatzAbstractions.io.DigitalInOutIO;
import frc.robot.CatzAbstractions.io.DigitalInOutIOInputsAutoLogged;

public class DigitalInOut {
	private final Debouncer debouncer;
	private final String name;
	private final boolean isInverted;

	private final DigitalInOutIO io;
	private final DigitalInOutIOInputsAutoLogged inputs = new DigitalInOutIOInputsAutoLogged();

	public DigitalInOut(DigitalInOutIO io, Time debounce, boolean isInverted, String name) {
		this.io = io;
		debouncer = new Debouncer(debounce.in(Units.Seconds), DebounceType.kBoth);
		this.isInverted = isInverted;
		this.name = name;
	}

	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs(name, inputs);
	}

	public boolean get() {
		return isInverted ? !inputs.value : inputs.value;
	}

	public boolean getDebounced() {
		return debouncer.calculate(get());
	}

	public boolean getDebouncedIfReal() {
		return Robot.isReal() && getDebounced();
	}

	public Command stateWait(boolean state) {
		return Commands.waitUntil(() -> get() == state);
	}

	public Command stateWaitWithDebounce(boolean state) {
		return Commands.waitUntil(() -> getDebounced() == state);
	}

	public Command stateWaitIfReal(boolean state, double waitSecondsSim) {
		return Commands.either(stateWait(state), Commands.waitSeconds(waitSecondsSim), () -> Robot.isReal());
	}

	public Command stateWaitWithDebounceIfReal(boolean state, double waitSecondsSim) {
		return Commands.either(
				stateWaitWithDebounce(state), Commands.waitSeconds(waitSecondsSim), () -> Robot.isReal());
	}



}
