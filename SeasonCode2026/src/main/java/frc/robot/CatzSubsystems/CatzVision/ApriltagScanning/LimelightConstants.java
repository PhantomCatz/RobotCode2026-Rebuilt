package frc.robot.CatzSubsystems.CatzVision.ApriltagScanning;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
public class LimelightConstants {

	public static final int kEnabledPipeline = 0;
	public static final int kDisabledPipeline = 1; //TODO try fiddling with pipelines through website
	public static final Vector<N3> enabledVisionStdDevs = VecBuilder.fill(0.3, 0.3, 99999.0);


	//offsets are forward/back, left/right, up/down
	// roll (along robot y axis), pitch (along robot x axis), yaw (along robot z axis)
	public static final ApriltagScanningIO[] LIMELIGHT_ARRAY = new ApriltagScanningIO[] {
		new ApriltagScanningIOMovable(new LimelightConfig("limelight-b",
			new Pose3d(new Translation3d(8.23178, -0.2794, 0.23114), new Rotation3d(0.0, 15.0, -18.0))
		))
	};


	public static final Translation2d TURRET_CENTER = new Translation2d(0.0, 0.0); //TODO Fill out
	public static final Distance TURRET_RADIUS = Units.Centimeters.of(0.0); //TODO Fill out

	public static class LimelightConfig {
		public String name = "no-name-assigned";
		public Pose3d robotToCameraOffset = new Pose3d();
		public Vector<N3> aprilTagVisionStdDevs = VecBuilder.fill(0.3, 0.3, 99999.0);

		public LimelightConfig(String name, Pose3d robotToCameraOffset){
			this.name = name;
			this.robotToCameraOffset = robotToCameraOffset;
		}

		public LimelightConfig(){}
	}

	//TODO Use this instead of vision shift for auto aim
	public static final int agreedTranslationUpdatesThreshold = 100;
	public static final Distance agreedTranslationUpdateEpsilon = Units.Centimeters.of(10.0);
}