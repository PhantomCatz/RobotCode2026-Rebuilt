package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Swipe_Outpost extends AutoRoutineBase {
    public Swipe_Outpost(){
        super("Swipe_Outpost");

        AutoTrajectory traj1 = getTrajectory("Swipe_Outpost",0);
        AutoTrajectory traj2 = getTrajectory("Swipe_Outpost",1);
        AutoTrajectory traj3 = getTrajectory("Swipe_Outpost",2);

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(0.5),
                    followTrajectoryWithAccuracy(traj1),
                    CatzSuperstructure.Instance.intakeON(),
                    Commands.waitSeconds(0.5),
                    followTrajectoryWithAccuracy(traj2)
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.intakeOFF(),
                    followTrajectoryWithAccuracy(traj3)
                ),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBallsNoJiggleNoStop(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT-2),
            shootAllBalls(AutonConstants.OUTPOST_SCORING_WAIT),
            Commands.print("done")
        );
    }
}
