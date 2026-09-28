package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Steal_Depot_Trench_Bump extends AutoRoutineBase {
    public Steal_Depot_Trench_Bump(){
        super("Steal_Depot_Trench_Bump");

        AutoTrajectory traj1 = getTrajectory("Steal_Depot_Trench_Bump",0);
        AutoTrajectory traj2 = getTrajectory("Steal_Depot_Trench_Bump",1);
        AutoTrajectory traj3 = getTrajectory("Steal_Depot_Trench_Bump",2);
        AutoTrajectory traj4 = getTrajectory("Steal_Depot_Trench_Bump",3);
        AutoTrajectory traj5 = getTrajectory("Steal_Depot_Trench_Bump",4);
        AutoTrajectory traj6 = getTrajectory("Steal_Depot_Trench_Bump",5);
        AutoTrajectory traj7 = getTrajectory("Steal_Depot_Trench_Bump",6);

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(2.5),
                    followTrajectoryWithAccuracy(traj1).alongWith(Commands.print("traj1")),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectoryWithAccuracy(traj2),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                    CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj3),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBallsNoJiggleNoStop(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT-2),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT-1.8),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj4),
                    CatzSuperstructure.Instance.intakeON()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj5),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            CatzSuperstructure.Instance.intakeOFF(),
            shootAllBallsNoJiggleNoStop(1.5),
            shootAllBalls(AutonConstants.OUTPOST_SCORING_WAIT),
            followTrajectoryWithAccuracy(traj6),
            CatzSuperstructure.Instance.intakeON(),
            followTrajectoryWithAccuracy(traj7),
            Commands.print("done")
        );
    }
}
