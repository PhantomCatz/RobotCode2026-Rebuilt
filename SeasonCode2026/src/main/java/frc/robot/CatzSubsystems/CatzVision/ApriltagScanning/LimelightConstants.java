package frc.robot.CatzSubsystems.CatzVision.ApriltagScanning;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

// Static import the specific units we need to keep code clean
import static edu.wpi.first.units.Units.*;

public class LimelightConstants {

    public static final int kEnabledPipeline = 0;
    public static final int kDisabledPipeline = 1;

    private static final double limelightCrosshairCompensationPitch = 0.0;
    private static final double limelightCrosshairCompensationYaw = 0.0; // -2.0 for sushi

    public static final ApriltagScanningIO[] LIMELIGHT_ARRAY = new ApriltagScanningIO[] {
        new ApriltagScanningIOMovable(new LimelightConfig("limelight-cheese",
            new Pose3d(
                new Translation3d(
                    Inches.of(2.25).in(Meters), //NOTE this offset is limelight's offset relative to the turret.
                    Inches.of(5.375).in(Meters),           //We recalculate robot's actual position based off of the data fed by the limelight.
                    Inches.of(21.125).in(Meters) //This makes it easy to account for latency.
                ),
                new Rotation3d(
                    Degrees.of(-1.0).in(Radians),
                    Degrees.of(14.0 + limelightCrosshairCompensationPitch).in(Radians),
                    Degrees.of(limelightCrosshairCompensationYaw).in(Radians)
                )
            )
        ))
    };

    // public static LoggedTunableNumber forward = new LoggedTunableNumber("Limelight/forward", -5.0);
    // public static LoggedTunableNumber leftward = new LoggedTunableNumber("Limelight/leftward", -1.25);
    // public static LoggedTunableNumber upward = new LoggedTunableNumber("Limelight/upward", 19.75);
    // public static LoggedTunableNumber pitch = new LoggedTunableNumber("Limpelight/pPitch", 18.0);
    // public static LoggedTunableNumber turretcenterx = new LoggedTunableNumber("Limelight/turretcenterx", -5.0);
    // public static LoggedTunableNumber turretcentery = new LoggedTunableNumber("Limelight/turretcentery", 5.0);
    // public static LoggedTunableNumber limelightx = new LoggedTunableNumber("Limelight/limelightx", 0.25);
    // public static LoggedTunableNumber limelighty = new LoggedTunableNumber("Limelight/limelighty", -6.5); //-13??

    public static class LimelightConfig {
        public String name = "no-name-assigned";
        public Pose3d robotToCameraOffset = new Pose3d();
        public Vector<N3> aprilTagVisionStdDevs = VecBuilder.fill(0.01, 0.01, 99999.0);

        public LimelightConfig(String name, Pose3d robotToCameraOffset){
            this.name = name;
            this.robotToCameraOffset = robotToCameraOffset;
        }
        public LimelightConfig(){}
    }
}
