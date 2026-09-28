package frc.robot.CatzAbstractions.Bases;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

public class DigitalInOut {
	private final Debouncer debouncer;
	private final String name;
	private final boolean isInverted;
	private final DigitalInput beambreak;

	public DigitalInOut(int id, double debounceTime, boolean isInverted, String name) {
		debouncer = new Debouncer(debounceTime, DebounceType.kBoth);
		this.isInverted = isInverted;
		this.name = name;
		this.beambreak = new DigitalInput(id);
	}

	public void periodic() {
		Logger.recordOutput("BeamBreak: " + name, get());
	}

	public boolean get() {
		return isInverted ? !beambreak.get() : beambreak.get();
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
