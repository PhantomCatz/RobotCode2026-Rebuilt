package frc.robot.CatzSubsystems.CatzVision.ApriltagScanning;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker.VisionObservation;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.TurretConstants;
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightConstants.LimelightConfig;
import frc.robot.Utilities.LimelightHelpers;
import frc.robot.Utilities.LimelightHelpers.PoseEstimate;

public class ApriltagScanningIOMovable implements ApriltagScanningIO{

    private Pose2d latestEstimate = new Pose2d();
	private Time latestEstimateTime = Units.Seconds.of(0.0);
    private int latestEstimateNumTags = 0;
	private LimelightConfig config = new LimelightConfig();

    public ApriltagScanningIOMovable(LimelightConfig config){
        this.config = config;
    }

    @Override
    public void setLatestEstimate(PoseEstimate poseEstimate, int minTagNum){
        if(poseEstimate == null) return;
		// SmartDashboard.putNumber(config.name + "/Tag Count", poseEstimate.tagCount);
		// SmartDashboard.putNumber(config.name + "/FGPA Timestamp", Timer.getFPGATimestamp());
		// SmartDashboard.putNumber(
		// 		config.name + "/Estimate to FGPA Timestamp", Utils.fpgaToCurrentTime(poseEstimate.timestampSeconds));
        latestEstimateNumTags = poseEstimate.tagCount;
		if (poseEstimate.tagCount >= minTagNum) {
			latestEstimate = poseEstimate.pose;
			latestEstimateTime = Units.Seconds.of(poseEstimate.timestampSeconds);
			CatzRobotTracker.Instance.addVisionObservation(
                new VisionObservation(config.name, poseEstimate.pose, poseEstimate.timestampSeconds, LimelightConstants.enabledVisionStdDevs.times(poseEstimate.avgTagDist))
			);
		}
    }

    @Override
    public void update(){
        updateGyro();
        updateCameraOffset();
		setLatestEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name), 1);
    }

    @Override
	public void updateInputs(VisionIOInputs inputs){
		inputs.hasTargets = latestEstimateNumTags > 0;
		inputs.calculatedPose = latestEstimate;
	}

    private void updateGyro() {
		Rotation2d theta = CatzRobotTracker.Instance.getEstimatedPose().getRotation();
		LimelightHelpers.SetRobotOrientation(config.name, theta.getDegrees(), 0, 0, 0, 0, 0);
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

	public LimelightConfig getConfig(){
		return config;
	}

    private void updateCameraOffset(){
        // System.out.println("update camera offset");
        Pose3d newCameraOffset = calculateCurrentCameraOffset();
        Transform2d transform = new Transform2d(newCameraOffset.getX(), newCameraOffset.getY(), Rotation2d.fromRadians(newCameraOffset.getRotation().getZ()));
        Logger.recordOutput("Turret Camera New Offset", CatzRobotTracker.Instance.getEstimatedPose().transformBy(transform));
        LimelightHelpers.setCameraPose_RobotSpace(
            config.name,
            newCameraOffset.getX(),
            -newCameraOffset.getY(),
            newCameraOffset.getZ(),
            Math.toDegrees(newCameraOffset.getRotation().getX()),
            Math.toDegrees(newCameraOffset.getRotation().getY()),
            Math.toDegrees(newCameraOffset.getRotation().getZ())
        );
    }

    private Pose3d cameraPose() {
        return new Pose3d(new Translation3d(Units.Inches.of(LimelightConstants.forward.get()).in(Units.Meters),Units.Inches.of(LimelightConstants.leftward.get()).in(Units.Meters), Units.Inches.of(LimelightConstants.upward.get()).in(Units.Meters)), new Rotation3d(Units.Degrees.of(180.0).in(Units.Radians), Units.Degrees.of(LimelightConstants.pitch.get()).in(Units.Radians), Units.Degrees.of(0.0).in(Units.Radians)));
    }

    private Pose3d calculateCurrentCameraOffset() {
        Pose3d originalOffset = cameraPose();
        Angle turretAngle = Units.Rotations.of(CatzTurret.Instance.getPosition());

        Translation2d limelightOffsetFromTurretCenter = new Translation2d(Units.Inches.of(LimelightConstants.limelightx.get()), Units.Inches.of(LimelightConstants.limelighty.get())).rotateBy(new Rotation2d(turretAngle));//new Translation2d(
            //LimelightConstants.TURRET_RADIUS.in(Units.Meters),
            //new Rotation2d(turretAngle)
        //);

        Translation2d limelightPositionOnRobot = new Translation2d(Units.Inches.of(LimelightConstants.turretcenterx.get()), Units.Inches.of(LimelightConstants.turretcentery.get()))
            .plus(limelightOffsetFromTurretCenter);

        double deltaTurretAngle = turretAngle.in(Units.Radians) - TurretConstants.HOME_POSITION.in(Units.Radians);

        Rotation3d newRotation = new Rotation3d(
            originalOffset.getRotation().getX(),
            originalOffset.getRotation().getY(),
            originalOffset.getRotation().getZ() + deltaTurretAngle
        );

        return new Pose3d(
            limelightPositionOnRobot.getX(),
            limelightPositionOnRobot.getY(),
            originalOffset.getZ(),
            newRotation
        );
    }

    @Override
    public int getNumTags() {
        return latestEstimateNumTags;
    }

}
