package frc.robot.CatzSubsystems.CatzVision.ApriltagScanning;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
// Import the base Units class
import edu.wpi.first.units.measure.Distance;

// Static import the specific units we need to keep code clean
import static edu.wpi.first.units.Units.*;

public class LimelightConstants {

    public static final int kEnabledPipeline = 0;
    public static final int kDisabledPipeline = 1;
    public static final Vector<N3> enabledVisionStdDevs = VecBuilder.fill(0.3, 0.3, 99999.0);

    public static final ApriltagScanningIO[] LIMELIGHT_ARRAY = new ApriltagScanningIO[] {
        new ApriltagScanningIOMovable(new LimelightConfig("limelight-cheese",
            new Pose3d(
                new Translation3d(
                    Inches.of(18.125).in(Meters),
                    Inches.of(-5.0).in(Meters),
                    Inches.of(21.0).in(Meters)
                ),
                new Rotation3d(
                    Degrees.of(0.0).in(Radians),
                    Degrees.of(20.0).in(Radians), // 12.0
                    Degrees.of(0.0).in(Radians)
                )
            )
        ))
    };

    public static final Translation2d TURRET_CENTER = new Translation2d(
        Inches.of(5.0).in(Meters),
        Inches.of(5.0).in(Meters)
    );

    public static final Distance TURRET_RADIUS = Inches.of(6.5);

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

    public static final int agreedTranslationUpdatesThreshold = 100;
    public static final Distance agreedTranslationUpdateEpsilon = Centimeters.of(10.0);
}
