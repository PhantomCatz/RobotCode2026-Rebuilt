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
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzVision.Detection.Detection;

public class PIDDriveCmdFuel extends Command{

    private final ProfiledPIDController translationController;
    private final ProfiledPIDController rotationController;

    private final double GOAL_VELOCITY;

    private Pose2d goalPos;
    private boolean outOfTime = false;

    private final Translation2d TRENCH_POSE;

    /**
     * Only to be used in autonomous. Drives towards an initial target until it sees a fuel,
     * then it starts driving towards the largest clump of fuels.
     * Constantly calculates the approximate time it will take to go back to the trench.
     * This command finishes when it has just enough time to go back to the trench to shoot.
     *
     * @param initialGoal Initial position to drive towards
     * @param goalVel The goal velocity when driving back to the trench
     */
    public PIDDriveCmdFuel(Pose2d initialGoal, double goalVel){
        addRequirements(CatzDrivetrain.Instance);

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
        this.TRENCH_POSE = FieldConstants.getTrenchShootingLocation();
    }

    @Override
    public void initialize(){
        Logger.recordOutput("PID Target Pose", goalPos);
    }

    @Override
    public void execute(){
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

        ChassisSpeeds goalChassisSpeeds = new ChassisSpeeds(targetVel * direction.getCos(), targetVel * direction.getSin(), targetOmega);
        CatzDrivetrain.Instance.drive(goalChassisSpeeds);

        if(DriverStation.isAutonomous()){
            double avgVel = (targetVel + GOAL_VELOCITY) / 2.0;
            double timeToReachTrench = currentPose.getTranslation().getDistance(TRENCH_POSE) / avgVel;
            if (timeToReachTrench < Robot.autonStartTime + 20.0 - AutonConstants.RETURN_TIME_BUFFER - Timer.getFPGATimestamp()) {
                outOfTime = true;
            }
        }
    }

    @Override
    public boolean isFinished(){
        return outOfTime;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("finished!!!!!! yayayay " + interrupted);
    }
}
