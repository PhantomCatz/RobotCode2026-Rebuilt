package frc.robot.CatzSubsystems.CatzVision.ApriltagScanning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightConstants.LimelightConfig;
import frc.robot.Utilities.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {

	public static LimelightSubsystem Instance = new LimelightSubsystem();

	private final ApriltagScanningIO[] ios;

	private LimelightSubsystem() {
		ios = LimelightConstants.LIMELIGHT_ARRAY;

		if (Robot.isReal()) {
			for(ApriltagScanningIO limelight : LimelightConstants.LIMELIGHT_ARRAY){
				LimelightConfig config = limelight.getConfig();

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
			}
		}
	}

	// These are the zones. Red alliance on left. Seperated by middle line and trench bar.
	// 0 1 2 3
	// 4 5 6 7
	private int getZone() {
		Pose2d pose = CatzRobotTracker.Instance.getEstimatedPose();
		if (pose.getY() > FieldConstants.fieldYHalf) {
			if (pose.getX() < FieldConstants.fieldTrenchX) {
				return 0;
			}
			if (pose.getX() < FieldConstants.fieldXHalf) {
				return 1;
			}
			if (pose.getX() < FieldConstants.fieldLength - FieldConstants.fieldTrenchX) {
				return 2;
			}
			return 3;
		}
		else {
			if (pose.getX() < FieldConstants.fieldTrenchX) {
				return 4;
			}
			if (pose.getX() < FieldConstants.fieldXHalf) {
				return 5;
			}
			if (pose.getX() < FieldConstants.fieldLength - FieldConstants.fieldTrenchX) {
				return 6;
			}
			return 7;
		}
	}

	@Override
	public void periodic() {
		for(int i = 0; i < ios.length; i++){
			ios[i].update();
			ios[i].setPipelineIndex(getZone());
		}
	}

	public boolean isSeeingApriltag(){
		for(ApriltagScanningIO io : ios){
			return io.getNumTags() > 0;
		}
		return false;
	}
}
