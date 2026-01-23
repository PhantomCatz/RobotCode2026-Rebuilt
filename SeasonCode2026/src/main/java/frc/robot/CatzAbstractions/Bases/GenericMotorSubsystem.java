package frc.robot.CatzAbstractions.Bases;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.google.common.base.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.Utilities.Setpoint;

public abstract class GenericMotorSubsystem<S extends GenericMotorIO<I>, I extends GenericMotorIO.MotorIOInputs> extends SubsystemBase {
	protected final S io;
	protected final I inputs;
	protected final String name;
	protected Setpoint setpoint = Setpoint.withBrakeSetpoint();

	public GenericMotorSubsystem(S io, I inputs, String name) {
		super(name);
		this.io = io;
		this.inputs = inputs;
		this.name = name;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs(name, (LoggableInputs) inputs);
	}

	public void applySetpoint(Setpoint setpoint) {
		this.setpoint = setpoint;
		setpoint.apply(io);
	}

	/**
	 * Creates a continous command for the subsystem to repeatedly go to a supplied
	 * setpoint.
	 *
	 * @param setpoint Supplier of setpoint to go to.
	 * @return Continuous Command for the subsystem.
	 */
	public Command followSetpointCommand(Supplier<Setpoint> supplier) {
		return run(() -> applySetpoint(supplier.get()));
	}

	public Command setpointCommand(Setpoint setpoint){
		return runOnce(() -> applySetpoint(setpoint));
	}

	public Command setpointCommand(Supplier<Setpoint> supplier){
		return runOnce(() -> applySetpoint(supplier.get()));
	}

	public Setpoint getSetpoint() {
		return setpoint;
	}

	public AngularVelocity getVelocity() {
		return Units.RotationsPerSecond.of(inputs.velocityRPS);
	}

	public double getPosition() {
		return inputs.position;
	}

	public double[] getSupplyCurrent() {
		return inputs.supplyCurrentAmps;
	}

	public double getAcceleration() { // TODO make this an array as well
		return inputs.accelerationRPS;
	}

	public double[] getTemp() {
		return inputs.tempCelcius;
	}

	public double[] getAppliedVoltage() {
		return inputs.appliedVolts;
	}
}
