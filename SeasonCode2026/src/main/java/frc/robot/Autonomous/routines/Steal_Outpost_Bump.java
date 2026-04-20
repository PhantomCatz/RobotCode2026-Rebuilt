package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Steal_Outpost_Bump extends AutoRoutineBase {
    public Steal_Outpost_Bump(){
        super("Steal_Outpost_Bump");

        AutoTrajectory traj1 = getTrajectory("Steal_Outpost_Bump",0);
        AutoTrajectory traj2 = getTrajectory("Steal_Outpost_Bump",1);
        AutoTrajectory traj3 = getTrajectory("Steal_Outpost_Bump",2);

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
            shootAllBallsNoJiggleNoStop(1.5),
            shootAllBalls(AutonConstants.OUTPOST_SCORING_WAIT),
            Commands.print("done")
        );
    }
}
