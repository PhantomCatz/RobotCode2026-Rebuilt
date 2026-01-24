package frc.robot.CatzSubsystems.CatzVision.ApriltagScanning;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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

	@Override
	public void periodic() {
		for(int i = 0; i < ios.length; i++){
			ios[i].update();
		}
	}

	public boolean isSeeingApriltag(){
		for(ApriltagScanningIO io : ios){
			return io.getNumTags() > 0;
		}
		return false;
	}
}