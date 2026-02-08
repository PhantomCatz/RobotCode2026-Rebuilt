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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzVision.Detection.Detection;

public class PIDDriveCmdCoral extends Command{

    private final ProfiledPIDController translationController;
    private final ProfiledPIDController rotationController;

    private final double POSITION_TOLERANCE_METERS = 0.3;
    private final double VELOCITY_TOLERANCE_MPS = 0.1;
    private final double ANGLE_TOLERANCE_DEGREES = 3.0;
    private final double ALLOWABLE_VISION_ADJUST = 4e-3; //TODO tune
    private final double GOAL_VELOCITY;

    private Pose2d goalPos;
    private boolean readyToScore = false;
    private boolean outOfTime = false;

    public PIDDriveCmdCoral(Pose2d goal, double goalVel){
        addRequirements(CatzDrivetrain.Instance);

        CatzDrivetrain.Instance.setPIDGoalPose(goal);

        // Configure the translation controller
        var translationConstraints = new TrapezoidProfile.Constraints(
            4.0,
            4.0
        );
        this.translationController = new ProfiledPIDController(3.0, 0.0, 0.0, translationConstraints);

        // Configure the rotation controller
        var rotationConstraints = new TrapezoidProfile.Constraints(
            360.0,
            720.0
        );
        this.rotationController = new ProfiledPIDController(3.0, 0.0, 0.0, rotationConstraints);
        this.rotationController.enableContinuousInput(-180.0, 180.0);
        this.GOAL_VELOCITY = goalVel;
    }

    @Override
    public void initialize(){
        goalPos = CatzDrivetrain.Instance.getPIDGoalPose();
        Logger.recordOutput("PID Target Pose", goalPos);
    }

    @Override
    public void execute(){
        if(readyToScore) return;
        if (Detection.Instance.getNearestGroupPose() != null) goalPos = Detection.Instance.getNearestGroupPose();
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
        Translation2d poseError = goalPos.minus(currentPose).getTranslation();

        if(poseError.getNorm() < 0.0001) {
            System.out.println("Pose error very small!!");
            return;
        }

        double currentDistance = poseError.getNorm();
        Logger.recordOutput("Current Distance", currentDistance);
        Rotation2d direction = poseError.getAngle();
        double angleError = MathUtil.inputModulus(goalPos.getRotation().getDegrees() - currentPose.getRotation().getDegrees(), -180.0, 180.0);
        // The goal of the translation controller is to drive the distance error to zero
        double targetVel = Math.abs(translationController.calculate(currentDistance, 0.0));
        // The goal of the rotation controller is to drive the angle to the target angle
        double targetOmega = -Math.toRadians(rotationController.calculate(angleError, 0.0));
        // Logger.recordOutput("Pose Error Omega", targetOmega);

        ChassisSpeeds goalChassisSpeeds = new ChassisSpeeds(targetVel * direction.getCos(), targetVel * direction.getSin(), targetOmega);
        CatzDrivetrain.Instance.drive(goalChassisSpeeds);

        CatzDrivetrain.Instance.setDistanceError(currentDistance);
        if(DriverStation.isAutonomous()){
            double avgVel = (targetVel + GOAL_VELOCITY) / 2.0;
            double timeToReachTrench = currentDistance / avgVel;
            if (timeToReachTrench < Robot.autonStartTime + 20.0 - AutonConstants.RETURN_TIME_BUFFER - Timer.getFPGATimestamp()) {
                outOfTime = true;
            }
        }
    }

    @Override
    public boolean isFinished(){
        return isAtTargetState() || outOfTime;
    }

    private boolean isAtTargetState(){
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
        ChassisSpeeds currentSpeed = CatzRobotTracker.Instance.getRobotChassisSpeeds();

        double distanceError = currentPose.getTranslation().getDistance(goalPos.getTranslation());
        double linearVelocity = Math.hypot(currentSpeed.vxMetersPerSecond, currentSpeed.vyMetersPerSecond);

        double rotationError = Math.abs(MathUtil.inputModulus(goalPos.getRotation().getDegrees() - currentPose.getRotation().getDegrees(), -180.0, 180.0));
        Logger.recordOutput("Rotation Error", rotationError < ANGLE_TOLERANCE_DEGREES);
        Logger.recordOutput("Distance Error", distanceError < POSITION_TOLERANCE_METERS);
        Logger.recordOutput("Linear Velocity", linearVelocity < VELOCITY_TOLERANCE_MPS);
        return distanceError < POSITION_TOLERANCE_METERS &&
               linearVelocity < VELOCITY_TOLERANCE_MPS &&
               rotationError < ANGLE_TOLERANCE_DEGREES;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("finished!!!!!! yayayay " + interrupted);
        CatzDrivetrain.Instance.drive(new ChassisSpeeds());
    }
}
