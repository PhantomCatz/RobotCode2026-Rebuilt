package frc.robot.CatzSubsystems.CatzVision.ApriltagScanning;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker.VisionObservation;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.TurretConstants;
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightConstants.LimelightConfig;
import frc.robot.Utilities.LimelightHelpers;
import frc.robot.Utilities.LimelightHelpers.PoseEstimate;

public class ApriltagScanningIOMovable implements ApriltagScanningIO {

    private Pose2d latestEstimate = new Pose2d();
    private Time latestEstimateTime = Units.Seconds.of(0.0);
    private int latestEstimateNumTags = 0;
    private LimelightConfig config = new LimelightConfig();

    public ApriltagScanningIOMovable(LimelightConfig config) {
        this.config = config;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.hasTargets = latestEstimateNumTags > 0;
        inputs.calculatedPose = latestEstimate;
    }

    @Override
    public void update() {
        // 1. Tell Limelight our Global Heading (Robot Gyro + Turret Angle)
        // This is required for MegaTag2 to work correctly on a turret.
        updateGyroWithTurret();

        // 2. Get the result (This result is the TURRET'S position on the field)
        // We do NOT update offsets here.
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name);
        setLatestEstimate(estimate, 1);
    }

    private void updateGyroWithTurret() {
        Rotation2d robotYaw = CatzRobotTracker.Instance.getEstimatedPose().getRotation();
        Rotation2d turretYaw = Rotation2d.fromRotations(CatzTurret.Instance.getLatencyCompensatedPosition());

        // Sum them to get the direction the "Camera Mount" structure is facing globally.
        double totalCameraHeading = robotYaw.plus(turretYaw).getDegrees();

        // Send to Limelight (yaw is index 0). Set rates to 0.
        LimelightHelpers.SetRobotOrientation(config.name, totalCameraHeading, 0, 0, 0, 0, 0);
    }

    @Override
    public void setLatestEstimate(PoseEstimate poseEstimate, int minTagNum) {
        if (poseEstimate == null) return;

        latestEstimateNumTags = poseEstimate.tagCount;

        if (poseEstimate.tagCount >= minTagNum) {
            double timestamp = poseEstimate.timestampSeconds;

            // --- LATENCY COMPENSATION LOGIC ---

            // 1. The Pose Limelight gave us (Where the TURRET was)
            Pose2d turretPoseFieldSpace = poseEstimate.pose;

            // 2. "Time Travel": Ask CatzTurret where it was at that exact timestamp
            double historicalTurretRad = CatzTurret.Instance.getAngleAtTime(timestamp);
            Rotation2d historicalTurretRot = new Rotation2d(historicalTurretRad);

            // 3. Define the Transform from Robot Center -> Turret Center
            //    This mimics the physical robot structure.
            Transform2d robotToTurret = new Transform2d(
                TurretConstants.TURRET_OFFSET, // e.g. Translation2d(0.2m, 0.0m)
                historicalTurretRot
            );

            // 4. Calculate where the Robot was:
            //    Robot = Turret * (Robot->Turret)^-1
            Pose2d robotPoseFieldSpace = turretPoseFieldSpace.transformBy(robotToTurret.inverse());

            // ----------------------------------

            latestEstimate = robotPoseFieldSpace;
            latestEstimateTime = Units.Seconds.of(timestamp);

            CatzRobotTracker.Instance.addVisionObservation(
                new VisionObservation(
                    config.name,
                    robotPoseFieldSpace, // Use our corrected robot pose
                    timestamp,
                    LimelightConstants.enabledVisionStdDevs.times(poseEstimate.avgTagDist)
                )
            );
        }
    }

    public void updateConfig(LimelightConfig config) {
        this.config = config;
    }

    public Pose2d getLatestEstimate() {
        return latestEstimate;
    }

    public Time getLatestEstimateTime() {
        return latestEstimateTime;
    }

    public int getLatestEstimateNumTags() {
        return latestEstimateNumTags;
    }

    public LimelightConfig getConfig() {
        return config;
    }

    @Override
    public int getNumTags() {
        return latestEstimateNumTags;
    }

}
