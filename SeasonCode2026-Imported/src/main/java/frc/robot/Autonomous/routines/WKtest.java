package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import org.wpilib.command2.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class WKtest extends AutoRoutineBase {
    public WKtest(){
        super("WKtest");

        AutoTrajectory traj1 = getTrajectory("WKtest", 0);
        AutoTrajectory traj2 = getTrajectory("WKtest", 1);
        AutoTrajectory traj3 = getTrajectory("WKtest", 2);

        prepRoutine(traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    followTrajectoryWithAccuracy(traj1))
                ,CatzSuperstructure.Instance.trackStaticHub()
            ), Commands.deadline(
                    Commands.sequence(
                        CatzSuperstructure.Instance.toggleIntakeRollers(),
                        followTrajectoryWithAccuracy(traj2)),
                CatzSuperstructure.Instance.trackStaticHub()
            ), Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.toggleIntakeRollers(),
                    followTrajectoryWithAccuracy(traj3)),
                    shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT)
                ,CatzSuperstructure.Instance.trackStaticHub())
            );
    }
}
