package frc.robot.CatzAbstractions.Bases;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.Utilities.Util;

public abstract class FlywheelMotorSubsystem<S extends GenericMotorIO<I>, I extends GenericMotorIO.MotorIOInputs> extends GenericMotorSubsystem<S, I> {

	protected final AngularVelocity epsilonThreshold;

	public FlywheelMotorSubsystem(S io, I inputs, String name, AngularVelocity epsilonThreshold) {
		super(io, inputs, name);
		this.epsilonThreshold = epsilonThreshold;
	}
	/**
	 * Gets whether or not the subsystem is within an acceptable threshold of a provided velocity.
	 *
	 * @param velocity Velocity to check proximity to.
	 * @return Whether the subsystem is acceptably near the given velocity.
	 */
	public boolean nearVelocity(AngularVelocity velocity) {
		return Util.epsilonEquals(
				velocity.baseUnitMagnitude(), getVelocity().baseUnitMagnitude(), epsilonThreshold.baseUnitMagnitude());
	}

	/**
	 * Gets whether or not the subsystem is within an acceptable threshold of it's velocity setpoint.
	 *
	 * @return Whether the subsystem is acceptably near it's setpoint's velocity. Returns false if not in velcity coontrol mode.
	 */
	public boolean spunUp() {
		return nearVelocity(BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(getSetpoint().baseUnits))
				&& getSetpoint().mode.isVelocityControl();
	}
}
