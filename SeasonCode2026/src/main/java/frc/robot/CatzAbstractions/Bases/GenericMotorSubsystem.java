package frc.robot.CatzAbstractions.Bases;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.CatzAbstractions.io.MotorIOInputsAutoLogged;
import frc.robot.Utilities.Setpoint;

public abstract class GenericMotorSubsystem extends SubsystemBase {
	protected final GenericMotorIO io;
	protected final String name;

	private Setpoint setpoint = Setpoint.withBrakeSetpoint();

	protected final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

	public GenericMotorSubsystem(GenericMotorIO io, String name) {
		super(name);
		this.io = io;
		this.name = name;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs(name, inputs);

		if(setpoint != null){
			setpoint.apply(io);
		}

	}

	public void setCurrentPosition(double position) {
		io.setCurrentPosition(position);
	}

	public void applySetpoint(Setpoint setpoint) {
		this.setpoint = setpoint;
	}

	/**
	 * Creates a one time, instantaneus command for the subsystem to go to a given
	 * Setpoint.
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

	public void setDutyCycle(double dutyCycle) {
		io.setDutyCycleSetpoint(dutyCycle);
	}

	public Command setDutyCycleCommand(double dutyCycle) {
		return runOnce(() -> setDutyCycle(dutyCycle));
	}

	public double getSetpointInUnits() {
		return setpoint.baseUnits;
	}

	public double getVelocityRPS() {
		return inputs.velocityRPS;
	}

	public double getPosition() {
		return inputs.position;
	}

	public double[] getSupplyCurrent() {
		return inputs.supplyCurrentAmps;
	}

	public double getAcceleration() { //TODO make this an array as well
		return inputs.accelerationRPS;
	}

	public double[] getTemp() {
		return inputs.tempCelcius;
	}

	public double[] getAppliedVoltage() {
		return inputs.appliedVolts;
	}
}
