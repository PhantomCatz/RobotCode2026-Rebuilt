package frc.robot.CatzAbstractions.Bases;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.io.GenericIOSim;
import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;
import frc.robot.CatzAbstractions.io.MotorIOInputsAutoLogged;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal.MotorIOTalonFXConfig;
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
	}

	public void applySetpoint(Setpoint setpoint) {
		this.setpoint = setpoint;
		setpoint.apply(io);
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
	 * Creates a continous command for the subsystem to repeatedly go to a supplied
	 * setpoint.
	 *
	 * @param setpoint Supplier of setpoint to go to.
	 * @return Continuous Command for the subsystem.
	 */
	public Command followSetpointCommand(Supplier<Setpoint> supplier) {
		return run(() -> applySetpoint(supplier.get()));
	}

	public Setpoint getSetpoint() {
		return setpoint;
	}

	public AngularVelocity getVelocity() {
		return Units.RotationsPerSecond.of(inputs.velocityRPS);
	}

	public Angle getPosition() {
		return Units.Rotations.of(inputs.position);
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

	protected static GenericMotorIO getIOInstance(MotorIOTalonFXConfig motorConfig) {
		switch (CatzConstants.hardwareMode) {
			case REAL:
				System.out.println("Shooter Configured for Real");
				return new GenericTalonFXIOReal(motorConfig);
			case SIM:
				System.out.println("Shooter Configured for Simulation");
				return new GenericIOSim();
			default:
				System.out.println("Shooter Unconfigured; defaulting to simulation");
				return new GenericIOSim();
		}
	}
}
