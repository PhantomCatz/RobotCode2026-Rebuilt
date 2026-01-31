package frc.robot.Autonomous;

import java.util.Set;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
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
            new InstantCommand(() -> CatzRobotTracker.getInstance().resetPose(startTraj.getInitialPose().get()))
            .andThen(Commands.sequence(sequence))
        );
    }

    protected Command followTrajectoryWithAccuracy(AutoTrajectory traj){
        return Commands.defer(() ->
                                new FunctionalCommand
                                (
                                    () -> {CatzDrivetrain.getInstance().followChoreoTrajectoryInit(traj); traj.cmd().initialize();},
                                    traj.cmd()::execute,
                                    traj.cmd()::end,
                                    () -> isAtPose(traj)
                                ),
                                Set.of(CatzDrivetrain.getInstance())
                             );
    }

    // protected Command trajectoryToObjectDetection() {

    // }

    private boolean isAtPose(AutoTrajectory trajectory){
        boolean isAtTrans = translationIsFinished(trajectory, AutonConstants.ACCEPTABLE_DIST_METERS);
        boolean isAtRot = rotationIsFinished(trajectory, AutonConstants.ACCEPTABLE_ANGLE_DEG);
        // System.out.println((isAtTrans && isAtRot));
        return isAtTrans && isAtRot;
    }

    private boolean rotationIsFinished(AutoTrajectory trajectory, double epsilonAngleDeg){
        Rotation2d curRot = CatzRobotTracker.getInstance().getEstimatedPose().getRotation();
        Rotation2d goalRot = trajectory.getFinalPose().get().getRotation();
        return Math.abs(goalRot.minus(curRot).getDegrees()) % 360 < epsilonAngleDeg;
    }

    private boolean translationIsFinished(AutoTrajectory trajectory, double epsilonDist) {
		Pose2d currentPose = CatzRobotTracker.getInstance().getEstimatedPose();
		Pose2d finalPose = trajectory.getFinalPose().get();
        // System.out.println((currentPose.getTranslation().getDistance(finalPose.getTranslation())));

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
