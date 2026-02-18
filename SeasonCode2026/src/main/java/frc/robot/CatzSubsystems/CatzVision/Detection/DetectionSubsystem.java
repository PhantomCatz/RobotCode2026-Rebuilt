package frc.robot.CatzSubsystems.CatzVision.Detection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Utilities.LimelightHelpers;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightConstants.LimelightConfig;

import org.littletonrobotics.junction.Logger;

public class DetectionSubsystem<IO extends DetectionIOLimelight> extends SubsystemBase implements Runnable {
	protected final IO io;
	private final LimelightConfig config;
	private final DetectionIOInputsAutoLogged inputs = new DetectionIOInputsAutoLogged();

	public DetectionSubsystem(LimelightConfig config, IO io) {
		this.io = io;
		this.config = config;
		if (Robot.isReal()) {
			LimelightHelpers.setCameraPose_RobotSpace(
					config.name,
					config.robotToCameraOffset.getX(),
					config.robotToCameraOffset.getY(),
					config.robotToCameraOffset.getZ(),
					Units.radiansToDegrees(
							config.robotToCameraOffset.getRotation().getX()),
					Units.radiansToDegrees(
							config.robotToCameraOffset.getRotation().getY()),
					Units.radiansToDegrees(
							config.robotToCameraOffset.getRotation().getZ()));

			io.configLimelight(config);
		}
	}

	@Override
	public void periodic() {
		if(Robot.isReal()) {
			io.updateInputs(inputs);
			Logger.processInputs("RealInputs/Detection", inputs);
			outputTelemetry();
			Logger.recordOutput("Detection/nearestFuel", inputs.nearestFuel);
		}
	}

	public boolean getDisabled() {
		return io.getDisabled();
	}

	/**
	 * @return The closest Fuel pose.
	 */
	public Pose2d getFuelPose() {
		return io.getFuelPose();
	}

	public Pose2d getNearestGroupPose() {
		return io.getNearestGroupPose();
	}

	public void setNearestGroupPose() {
		io.setNearestGroupPose();
	}

	/**
	 * @param base The translation to evaluate the closest Fuel relative to
	 *
	 * @return A pose with a rotation component equal to the angle to face the Fuel and the translation component of the closest Fuel.
	 */
	public Pose2d getFuelTranslationAndPoint() {
		Translation2d t = getFuelPose().getTranslation();
		Rotation2d r = t.minus(CatzRobotTracker.Instance.getEstimatedPose().getTranslation()).getAngle();
		// LogUtil.recordPose2d("Detection PID/Fuel Translation And Point", new Pose2d(t, r));
		return new Pose2d(t, r);
	}

	public int fuelCount() {
		return io.fuelCount();
	}

	public void setPipeline(int index) {
		io.setPipeline(index);
	}

	public boolean hasFuel() {
		return getFuelPose() != null;
	}

	public void outputTelemetry() {
		// LoggedTracer.record(config.name);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty(config.name + "/Has Fuel", () -> hasFuel(), null);
		builder.addDoubleProperty(
				config.name + "/Latest Pipeline Index",
				() -> LimelightHelpers.getCurrentPipelineIndex(config.name),
				null);
	}

	@Override
	public void run() {
		while (true) {
			setNearestGroupPose();
		}
	}
}
