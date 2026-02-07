package frc.robot.Commands.DriveAndRobotOrientationCmds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
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

    private final boolean REQUIRES_ACCURACY;

    private Pose2d goalPos;

    public PIDDriveCmd(Pose2d goal, double goalVel, double velTolerance, double posTolerance, double angleTolerance, boolean requiresAccuracy) {
        addRequirements(CatzDrivetrain.Instance);
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

    public PIDDriveCmd(Pose2d goal, boolean requiresAccuracy) {
        addRequirements(CatzDrivetrain.Instance);
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

    @Override
    public void initialize(){
        Logger.recordOutput("PID Target Pose", goalPos);
    }

    @Override
    public void execute(){
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();

        Translation2d poseError = goalPos.minus(currentPose).getTranslation();
        double currentDistance = poseError.getNorm();

        Rotation2d direction = poseError.getAngle();
        double angleError = MathUtil.inputModulus(goalPos.getRotation().getDegrees() - currentPose.getRotation().getDegrees(), -180.0, 180.0);

        double targetVel = Math.max(Math.abs(translationController.calculate(currentDistance, 0.0)), GOAL_VELOCITY);

        double targetOmega = -Math.toRadians(rotationController.calculate(angleError, 0.0));
        ChassisSpeeds goalChassisSpeeds = new ChassisSpeeds(targetVel * direction.getCos(), targetVel * direction.getSin(), targetOmega);

        CatzDrivetrain.Instance.drive(goalChassisSpeeds);

        if(DriverStation.isAutonomous()){
            double avgVel = (targetVel + GOAL_VELOCITY) / 2.0;

            CatzDrivetrain.Instance.timeToReachTrench = currentDistance / avgVel;
        }
    }

    @Override
    public boolean isFinished(){
        boolean atTargetState = isAtTargetState();
        if(REQUIRES_ACCURACY){
            return  atTargetState && LimelightSubsystem.Instance.isSeeingApriltag() && CatzRobotTracker.Instance.getVisionPoseShift().getNorm() < ALLOWABLE_VISION_ADJUST;
        }else{
            return atTargetState;
        }
    }

    private boolean isAtTargetState(){
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
        ChassisSpeeds currentSpeed = CatzRobotTracker.Instance.getRobotChassisSpeeds();

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
