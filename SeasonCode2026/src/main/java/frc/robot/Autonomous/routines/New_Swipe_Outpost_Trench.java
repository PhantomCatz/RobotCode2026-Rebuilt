package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class New_Swipe_Outpost_Trench extends AutoRoutineBase {
    public New_Swipe_Outpost_Trench(){
        super("New_Swipe_Outpost_Trench");

        AutoTrajectory traj1 = getTrajectory("New_Swipe_Outpost_Trench",0);
        AutoTrajectory traj2 = getTrajectory("New_Swipe_Outpost_Trench",1);
        AutoTrajectory traj3 = getTrajectory("New_Swipe_Outpost_Trench",2);
        AutoTrajectory traj4 = getTrajectory("New_Swipe_Outpost_Trench",3);

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectoryWithAccuracy(traj1),
                    CatzSuperstructure.Instance.intakeON(),
                    Commands.waitSeconds(1),
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
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT-1.8),
            followTrajectoryWithAccuracy(traj4),
            Commands.print("done")
        );
    }
}
