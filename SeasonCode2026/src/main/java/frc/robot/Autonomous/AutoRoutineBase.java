package frc.robot.Autonomous;

import java.util.Set;


import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer.CatzSpindexer;
import frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer.CatzYdexer;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels.CatzFlywheels;
import frc.robot.CatzSubsystems.CatzShooter.CatzHood.CatzHood;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;

public class AutoRoutineBase {
    private AutoRoutine routine;

    public AutoRoutineBase(String name) {
        routine = CatzConstants.autoFactory.newRoutine(name);
    }

    protected void prepRoutine(AutoTrajectory startTraj, Command... sequence) {
        routine.active().onTrue(
                new InstantCommand(() -> {
                    CatzRobotTracker.getInstance().resetPose(startTraj.getInitialPose().get());
                })
                        .andThen(Commands.sequence(sequence)));
    }

    protected Command shootAllBalls(double time){
        return Commands.sequence(
            Commands.print("shootAllBalls command"),
            CatzSuperstructure.Instance.cmdHubShoot().withTimeout(time),
            CatzSuperstructure.Instance.cmdShooterStop()
        ).deadlineFor(CatzSuperstructure.Instance.jiggleIntakeCommand())
        .andThen(CatzSuperstructure.Instance.intakeOFF()
        .andThen(CatzSuperstructure.Instance.deployIntake()));
    }

    protected Command shootAllBallsNoJiggle(double time){
        return Commands.sequence(
            Commands.print("shootAllBalls w/out jiggle command"),
            CatzSuperstructure.Instance.cmdHubShoot().withTimeout(time),
            CatzSuperstructure.Instance.cmdShooterStop()
        )
        .andThen(CatzSuperstructure.Instance.intakeOFF());
    }

    protected Command shootAllBallsNoJiggleNoStop(double time){
        return Commands.sequence(
            Commands.print("shootAllBalls noStop w/out jiggle command"),
            CatzSuperstructure.Instance.cmdHubShoot().withTimeout(time)
        )
        .andThen(CatzSuperstructure.Instance.intakeOFF());
    }

    protected Command shootAllBallsNoStop(double time){
        return Commands.sequence(
            Commands.print("shootAllBalls noStop command"),
            CatzSuperstructure.Instance.cmdHubShoot().withTimeout(time)
        ).deadlineFor(CatzSuperstructure.Instance.jiggleIntakeCommand())
        .andThen(CatzSuperstructure.Instance.intakeOFF()
        .andThen(CatzSuperstructure.Instance.deployIntake()));
    }

    private double pathStartTime = 0.0;
    protected Command followTrajectory(AutoTrajectory traj) {
        return Commands.defer(() -> {
            final Command choreoCommand = traj.cmd();
            return new FunctionalCommand(
                    () -> {
                        CatzDrivetrain.getInstance().followChoreoTrajectoryInit(traj);
                        choreoCommand.initialize();
                        pathStartTime = Timer.getFPGATimestamp();
                    },
                    choreoCommand::execute,
                    choreoCommand::end,
                    () -> isAtPose(traj)).withTimeout(traj.getRawTrajectory().getTotalTime());
        }, Set.of(CatzDrivetrain.getInstance()));
    }

    protected Command followSlowTrajectory(AutoTrajectory traj) {
        return Commands.defer(() -> {
            final Command choreoCommand = traj.cmd();
            return new FunctionalCommand(
                    () -> {
                        CatzDrivetrain.getInstance().followSlowChoreoTrajectoryInit(traj);
                        choreoCommand.initialize();
                        pathStartTime = Timer.getFPGATimestamp();
                    },
                    choreoCommand::execute,
                    choreoCommand::end,
                    () -> isAtPose(traj)).withTimeout(traj.getRawTrajectory().getTotalTime());
        }, Set.of(CatzDrivetrain.getInstance()));
    }

    protected Command followTrajectoryWithAccuracy(AutoTrajectory traj) {
        return Commands.sequence(
            // Initial trajectory following
            Commands.runOnce(()->pathStartTime = Timer.getFPGATimestamp()),
            traj.cmd(),

            new FunctionalCommand(
                    () -> {
                    },

                    () -> {
                            // Final pose adjustment
                            var finalPoseOpt = traj.getFinalPose();

                            if (finalPoseOpt.isPresent()) {
                                Pose2d finalPose = finalPoseOpt.get();

                                SwerveSample holdSample = new SwerveSample(
                                        traj.getRawTrajectory().getTotalTime(),
                                        finalPose.getX(),
                                        finalPose.getY(),
                                        finalPose.getRotation().getRadians(),
                                        0, 0, 0,
                                        0, 0, 0,
                                        new double[4], new double[4]);

                                CatzDrivetrain.getInstance().followChoreoTrajectoryExecute(holdSample);
                            }
                    },

                    (interrupted) -> CatzDrivetrain.getInstance().stopDriving(),

                    () -> isAtPose(traj),

                    CatzDrivetrain.getInstance()
            ).unless(() -> isAtStrictPose(traj))
        ).withTimeout(traj.getRawTrajectory().getTotalTime() + 5.0);
    }

    protected Command followTrajectoryWhileShooting(AutoTrajectory traj) {
        return Commands.defer(() -> {
            final Command choreoCommand = traj.cmd();
            return new FunctionalCommand(
                    () -> {
                        CatzDrivetrain.getInstance().followChoreoTrajectoryInit(traj);
                        choreoCommand.initialize();
                        pathStartTime = Timer.getFPGATimestamp();
                        // CatzDrivetrain.getInstance().setShootWhileMoveConfig();
                    },
                    () -> {
                        choreoCommand.execute();
                        CatzSuperstructure.Instance.shootWhileMove(true, true, CatzRobotTracker.Instance.getEstimatedPose(), CatzRobotTracker.Instance.getRobotRelativeChassisSpeeds());
                    },
                    choreoCommand::end,
                    () -> isAtStrictPose(traj)).withTimeout(traj.getRawTrajectory().getTotalTime() + 5);
        }, Set.of(CatzDrivetrain.getInstance(), CatzTurret.Instance, CatzFlywheels.Instance, CatzHood.Instance, CatzSpindexer.Instance, CatzYdexer.Instance))
        .andThen(CatzSuperstructure.Instance.cmdShooterStop());
    }

    private boolean isAtPose(AutoTrajectory trajectory) {
        boolean isAtTrans = translationIsFinished(trajectory, AutonConstants.ACCEPTABLE_DIST_METERS);
        boolean isAtRot = rotationIsFinished(trajectory, AutonConstants.ACCEPTABLE_ANGLE_DEG);
        // System.out.println((isAtTrans && isAtRot));
        return isAtTrans && isAtRot && (Timer.getFPGATimestamp() - pathStartTime > trajectory.getRawTrajectory().getTotalTime()/2.0);
    }

    private boolean isAtStrictPose(AutoTrajectory trajectory) {
        boolean isAtTrans = translationIsFinished(trajectory, AutonConstants.ACCEPTABLE_STRICT_DIST_METERS);
        boolean isAtRot = rotationIsFinished(trajectory, AutonConstants.ACCEPTABLE_STRICT_ANGLE_DEG);

        return isAtTrans && isAtRot && (Timer.getFPGATimestamp() - pathStartTime > trajectory.getRawTrajectory().getTotalTime()/2.0);
    }

    private boolean rotationIsFinished(AutoTrajectory trajectory, double epsilonAngleDeg) {
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

    protected AutoTrajectory getTrajectory(String name) {
        return routine.trajectory(name);
    }

    protected AutoTrajectory getTrajectory(String name, int index) {
        return routine.trajectory(name, index);
    }

    public AutoRoutine getRoutine() {
        return routine;
    }
}
