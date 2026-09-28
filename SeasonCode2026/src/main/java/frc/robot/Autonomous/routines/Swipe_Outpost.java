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
        AutoTrajectory traj4 = getTrajectory("Swipe_Outpost",2);

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    followTrajectoryWithAccuracy(traj1),
                    CatzSuperstructure.Instance.intakeON(),
                    Commands.waitSeconds(2),
                    followTrajectoryWithAccuracy(traj2),
                    Commands.waitSeconds(1)
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
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT-1.8),
            followTrajectoryWithAccuracy(traj4),
            Commands.print("done")
        );
    }
}
