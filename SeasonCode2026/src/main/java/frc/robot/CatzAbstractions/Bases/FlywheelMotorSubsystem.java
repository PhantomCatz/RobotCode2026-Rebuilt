package frc.robot.CatzAbstractions.Bases;

import frc.robot.CatzAbstractions.io.GenericMotorIO;
import frc.robot.Utilities.DelayedBoolean;
import frc.robot.Utilities.Setpoint;

public abstract class FlywheelMotorSubsystem extends GenericMotorSubsystem {
	protected final double epsilonThreshold;

	private boolean mHoming = false;
	private boolean mNeedsToHome = true;
	private DelayedBoolean mHomingDelay;

	public FlywheelMotorSubsystem(GenericMotorIO io, String name, double epsilonThreshold) {
		super(io, name);
		this.epsilonThreshold = epsilonThreshold;
	}

	@Override
	public void periodic() {
		super.periodic();
	}
}
