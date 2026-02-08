package frc.robot.Autonomous;


import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;

public class AutoRoutineBase {
    private AutoRoutine routine;

    public AutoRoutineBase(String name){
        routine = CatzConstants.autoFactory.newRoutine(name);
    }

    protected void prepRoutine(AutoTrajectory startTraj, Command... sequence){
        routine.active().onTrue(
            new InstantCommand(() -> CatzRobotTracker.Instance.resetPose(startTraj.getInitialPose().get()))
            .andThen(Commands.sequence(sequence))
        );
    }

    protected Command followTrajectoryWithAccuracy(AutoTrajectory traj) {
        return Commands.sequence(
            // Step 1: Run the standard Choreo path.
            // This handles the motion perfectly until the timer runs out.
            traj.cmd(),

            // Step 2: Run a "Hold" command.
            // This takes over immediately after the timer ends and forces correction.
            new FunctionalCommand(
                () -> {}, // Init: No setup needed, robot is already moving near target

                // Execute: Manually construct a "Stationary" sample at the target pose
                () -> {
                    // getFinalPose() automatically handles Alliance Flipping for us!
                    var finalPoseOpt = traj.getFinalPose();

                    if (finalPoseOpt.isPresent()) {
                        Pose2d finalPose = finalPoseOpt.get();

                        // Create a synthetic sample representing "Standing Still at Target"
                        // We use the SwerveSample constructor (or you can modify your execute
                        // method to take a Pose2d directly for holding).
                        // Assuming standard Choreo SwerveSample constructor:
                        SwerveSample holdSample = new SwerveSample(
                            traj.getRawTrajectory().getTotalTime(), // t
                            finalPose.getX(),                       // x
                            finalPose.getY(),                       // y
                            finalPose.getRotation().getRadians(),   // heading
                            0, 0, 0,                                // vx, vy, omega (STOPPED)
                            0, 0, 0,                                // ax, ay, alpha (NO ACCEL)
                            new double[4], new double[4]            // Module forces (optional)
                        );

                        CatzDrivetrain.Instance.followChoreoTrajectoryExecute(holdSample);
                    }
                },

                // End: Stop the robot when we are finally done
                (interrupted) -> CatzDrivetrain.Instance.stopDriving(),

                // IsFinished: NOW we check your custom accuracy logic
                () -> isAtPose(traj),

                // Requirements
                CatzDrivetrain.Instance
            )
        );
    }

    private boolean isAtPose(AutoTrajectory trajectory){
        boolean isAtTrans = translationIsFinished(trajectory, AutonConstants.ACCEPTABLE_DIST_METERS);
        boolean isAtRot = rotationIsFinished(trajectory, AutonConstants.ACCEPTABLE_ANGLE_DEG);

        return isAtTrans && isAtRot;
    }

    private boolean rotationIsFinished(AutoTrajectory trajectory, double epsilonAngleDeg){
        Rotation2d curRot = CatzRobotTracker.Instance.getEstimatedPose().getRotation();
        Rotation2d goalRot = trajectory.getFinalPose().get().getRotation();

        return Math.abs(goalRot.minus(curRot).getDegrees()) % 360 < epsilonAngleDeg;
    }

    private boolean translationIsFinished(AutoTrajectory trajectory, double epsilonDist) {
		Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
		Pose2d finalPose = trajectory.getFinalPose().get();


		return currentPose.getTranslation().getDistance(finalPose.getTranslation()) < epsilonDist;
	}

    protected AutoTrajectory getTrajectory(String name){
        return routine.trajectory(name);
    }

    protected AutoTrajectory getTrajectory(String name, int index){
        return routine.trajectory(name, index);
    }

    public AutoRoutine getRoutine(){
        return routine;
    }
}
