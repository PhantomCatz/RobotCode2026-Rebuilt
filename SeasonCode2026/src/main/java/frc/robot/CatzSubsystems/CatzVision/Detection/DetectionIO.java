package frc.robot.CatzSubsystems.CatzVision.Detection;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;

public abstract class DetectionIO {
	@AutoLog
	public static class DetectionIOInputs {
		public Pose2d nearestFuel;
		public Pose2d nearestGroupFuel;
	}
	protected boolean disabled = Robot.isSimulation();

	public void disable(boolean disable) {
		disabled = disable;
	}

	public abstract void updateInputs(DetectionIOInputs inputs);

	public abstract Pose2d getFuelPose();

	public abstract Pose2d getNearestGroupPose();

	public abstract void setNearestGroupPose();

	public boolean getDisabled() {
		return disabled;
	}

	public abstract boolean txComplete(double tx);

	public abstract int fuelCount();

	public abstract Translation2d calcDistToFuel(double tx, double ty);

	public abstract void setPipeline(int index);
}
