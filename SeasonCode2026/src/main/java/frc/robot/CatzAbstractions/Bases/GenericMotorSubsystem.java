package frc.robot.CatzAbstractions.Bases;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.BaseStatusSignal;
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

	/**
	 * Applies a setpoint to the subsytem.
	 * 
	 * @param setpoint Setpoint to apply
	 */
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

	/**
	 * Applies a setpoint as a command.
	 * 
	 * @param setpoint setpoint to be applied
	 * @return command for the setpoint to be applied
	 */
	public Command setpointCommand(Setpoint setpoint){
		return runOnce(() -> applySetpoint(setpoint));
	}

	/**
	 * Applies a supplier of a setpoint as a command.
	 * 
	 * @param setpoint setpoint to be applied
	 * @return command for the setpoint to be applied
	 */
	public Command setpointCommand(Supplier<Setpoint> supplier){
		return runOnce(() -> applySetpoint(supplier.get()));
	}

	/**
	 * Sets gains P and V
	 * 
	 * @param p proportional gain kP. Corrects error by a factor of the current offset
	 * @param v velocity feedforward kV. Voltage to maintain a target velocity
	 */
	public void setGainsPV(double p, double v){
		io.setGainsSlot0(p, 0.0, 0.0, 0.0, v, 0.0, 0.0);
	}

	/**
	 * Returns the base status signals of the IO.
	 * 
	 * @return the base status signals of the IO
	 */
	public BaseStatusSignal[] getSignals(){
		return io.getSignals();
	}

	/**
	 * Returns the subsytems current target setpoint.
	 * 
	 * @return the subsytems current target setpoint
	 */
	public Setpoint getSetpoint() {
		return setpoint;
	}

	/**
	 * Returns the subsytems angular velocity as an AngularVelocity object.
	 * 
	 * @return the subsytems angular velocity as an AngularVelocity object
	 */
	public AngularVelocity getVelocity() {
		return Units.RotationsPerSecond.of(inputs.velocityRPS);
	}


	/**
	 * Returns the subsytems position, compensated with latency.
	 * 
	 * @return the subsytems position, compensated with latency
	 */
	public double getLatencyCompensatedPosition() {
		return inputs.position;
	}


	/**
	 * Returns the current and past supply current of the subsytem as an array of doubles.
	 * 
	 * @return
	 */
	public double[] getSupplyCurrent() {
		return inputs.supplyCurrentAmps;
	}

	public double getAcceleration() {
		return inputs.accelerationRPS;
	}

	public double[] getTemp() {
		return inputs.tempCelcius;
	}

	public double[] getAppliedVoltage() {
		return inputs.appliedVolts;
	}
}
