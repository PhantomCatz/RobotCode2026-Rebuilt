package frc.robot.Commands.DriveAndRobotOrientationCmds;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightSubsystem;

public class PIDDriveCmd extends Command {
    private final ProfiledPIDController translationController;
    private final ProfiledPIDController rotationController;

    private final double POSITION_TOLERANCE_METERS;
    private final double VELOCITY_TOLERANCE_MPS;
    private final double ANGLE_TOLERANCE_DEGREES;
    private final double ALLOWABLE_VISION_ADJUST;
    private final double GOAL_VELOCITY;
    private double waitTime = 0.0;

    private final boolean REQUIRES_ACCURACY;

    private Pose2d goalPos;

    public PIDDriveCmd(Pose2d goal, double goalVel, double velTolerance, double posTolerance, double angleTolerance, boolean requiresAccuracy) {
        addRequirements(CatzDrivetrain.getInstance());
        this.goalPos = goal;

        this.POSITION_TOLERANCE_METERS = posTolerance;
        this.VELOCITY_TOLERANCE_MPS = velTolerance;
        this.ANGLE_TOLERANCE_DEGREES = angleTolerance;
        this.ALLOWABLE_VISION_ADJUST = 4e-3;
        this.REQUIRES_ACCURACY = requiresAccuracy;
        this.GOAL_VELOCITY = goalVel;

        // Configure the translation controller
        var translationConstraints = new TrapezoidProfile.Constraints(
                4.0,
                4.0);
        this.translationController = new ProfiledPIDController(3.0, 0.0, 0.0, translationConstraints);

        // Configure the rotation controller
        var rotationConstraints = new TrapezoidProfile.Constraints(
                360.0,
                720.0);
        this.rotationController = new ProfiledPIDController(3.0, 0.0, 0.0, rotationConstraints);
        this.rotationController.enableContinuousInput(-180.0, 180.0);

    }

    public PIDDriveCmd(Pose2d goal, boolean requiresAccuracy, double positionToleranceMeters) {
        addRequirements(CatzDrivetrain.getInstance());
        this.goalPos = goal;

        this.REQUIRES_ACCURACY = requiresAccuracy;
        this.POSITION_TOLERANCE_METERS = positionToleranceMeters;
        this.VELOCITY_TOLERANCE_MPS = 0.1;
        this.ANGLE_TOLERANCE_DEGREES = 5.0;
        this.ALLOWABLE_VISION_ADJUST = 4e-3;
        this.GOAL_VELOCITY = 0.0;

        // Configure the translation controller
        var translationConstraints = new TrapezoidProfile.Constraints(
                4.0,
                4.0);
                this.translationController = new ProfiledPIDController(3.0, 0.0, 0.0, translationConstraints);

        // Configure the rotation controller
        var rotationConstraints = new TrapezoidProfile.Constraints(
                360.0,
                720.0);
        this.rotationController = new ProfiledPIDController(3.0, 0.0, 0.0, rotationConstraints);
        this.rotationController.enableContinuousInput(-180.0, 180.0);

    }

    public PIDDriveCmd(Pose2d goal, boolean requiresAccuracy) {
        addRequirements(CatzDrivetrain.getInstance());
        this.goalPos = goal;

        this.REQUIRES_ACCURACY = requiresAccuracy;
        this.POSITION_TOLERANCE_METERS = 0.02;
        this.VELOCITY_TOLERANCE_MPS = 0.1;
        this.ANGLE_TOLERANCE_DEGREES = 3.0;
        this.ALLOWABLE_VISION_ADJUST = 4e-3;
        this.GOAL_VELOCITY = 0.0;

        // Configure the translation controller
        var translationConstraints = new TrapezoidProfile.Constraints(
                4.0,
                4.0);
                this.translationController = new ProfiledPIDController(3.0, 0.0, 0.0, translationConstraints);

        // Configure the rotation controller
        var rotationConstraints = new TrapezoidProfile.Constraints(
                360.0,
                720.0);
        this.rotationController = new ProfiledPIDController(3.0, 0.0, 0.0, rotationConstraints);
        this.rotationController.enableContinuousInput(-180.0, 180.0);

    }

    public PIDDriveCmd(double waitTime, Pose2d goal, boolean requiresAccuracy) {
        addRequirements(CatzDrivetrain.getInstance());
        this.goalPos = goal;
        this.waitTime = waitTime;

        this.REQUIRES_ACCURACY = requiresAccuracy;
        this.POSITION_TOLERANCE_METERS = 0.02;
        this.VELOCITY_TOLERANCE_MPS = 0.1;
        this.ANGLE_TOLERANCE_DEGREES = 3.0;
        this.ALLOWABLE_VISION_ADJUST = 4e-3;
        this.GOAL_VELOCITY = 0.0;

        // Configure the translation controller
        var translationConstraints = new TrapezoidProfile.Constraints(
                4.0,
                4.0);
                this.translationController = new ProfiledPIDController(3.0, 0.0, 0.0, translationConstraints);

        // Configure the rotation controller
        var rotationConstraints = new TrapezoidProfile.Constraints(
                360.0,
                720.0);
        this.rotationController = new ProfiledPIDController(3.0, 0.0, 0.0, rotationConstraints);
        this.rotationController.enableContinuousInput(-180.0, 180.0);

    }

    double startTime = 0.0;

    @Override
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
        Logger.recordOutput("PID Target Pose", goalPos);

        Pose2d currentPose = CatzRobotTracker.getInstance().getEstimatedPose();
        double currentDistance = goalPos.getTranslation().getDistance(currentPose.getTranslation());
        translationController.reset(currentDistance);
        double initAngleError = MathUtil.inputModulus(goalPos.getRotation().getDegrees() - currentPose.getRotation().getDegrees(), -180.0, 180.0);
        rotationController.reset(initAngleError);
    }

    @Override
    public void execute(){
        Pose2d currentPose = CatzRobotTracker.getInstance().getEstimatedPose();

        Translation2d poseError = goalPos.minus(currentPose).getTranslation();
        double currentDistance = poseError.getNorm();
        Rotation2d direction = poseError.getAngle();

        double translationFeedback = translationController.calculate(currentDistance, 0.0);
        double translationFeedforward = translationController.getSetpoint().velocity;
        double targetVel = Math.max(Math.abs(translationFeedback + translationFeedforward), GOAL_VELOCITY);

        double angleError = MathUtil.inputModulus(goalPos.getRotation().getDegrees() - currentPose.getRotation().getDegrees(), -180.0, 180.0);
        double rotationFeedback = rotationController.calculate(angleError, 0.0);
        double rotationFeedforward = rotationController.getSetpoint().velocity;

        double targetOmega = -Math.toRadians(rotationFeedback + rotationFeedforward);

        ChassisSpeeds goalChassisSpeeds = new ChassisSpeeds(
            targetVel * direction.getCos(),
            targetVel * direction.getSin(),
            targetOmega
        );
        CatzDrivetrain.getInstance().drive(goalChassisSpeeds);

    }

    @Override
    public boolean isFinished(){
        boolean atTargetState = isAtTargetState();

        if(REQUIRES_ACCURACY){
            double curTime = Timer.getFPGATimestamp();
            return  atTargetState && LimelightSubsystem.Instance.isSeeingApriltag() && CatzRobotTracker.Instance.getVisionPoseShift().getNorm() < ALLOWABLE_VISION_ADJUST
                    && curTime - startTime > waitTime;
        }else{
            return atTargetState;
        }
    }

    private boolean isAtTargetState(){
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
        ChassisSpeeds currentSpeed = CatzRobotTracker.Instance.getRobotRelativeChassisSpeeds();

        double distanceError = currentPose.getTranslation().getDistance(goalPos.getTranslation());
        double linearVelocity = Math.hypot(currentSpeed.vxMetersPerSecond, currentSpeed.vyMetersPerSecond);

        double rotationError = Math.abs(MathUtil.inputModulus(goalPos.getRotation().getDegrees() - currentPose.getRotation().getDegrees(), -180.0, 180.0));

        return distanceError < POSITION_TOLERANCE_METERS &&
               linearVelocity < VELOCITY_TOLERANCE_MPS &&
               rotationError < ANGLE_TOLERANCE_DEGREES;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("finished!!!!!! yayayay " + interrupted);
    }
}
