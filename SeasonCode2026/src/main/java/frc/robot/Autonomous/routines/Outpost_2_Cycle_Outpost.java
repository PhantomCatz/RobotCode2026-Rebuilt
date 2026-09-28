package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Outpost_2_Cycle_Outpost extends AutoRoutineBase {
    public Outpost_2_Cycle_Outpost(){
        super("Outpost_2_Cycle_Outpost");

        AutoTrajectory traj1 = getTrajectory("Outpost_2_Cycle_Outpost",0);
        AutoTrajectory traj2 = getTrajectory("Outpost_2_Cycle_Outpost",1);
        AutoTrajectory traj3 = getTrajectory("Outpost_2_Cycle_Outpost",2);
        AutoTrajectory traj4 = getTrajectory("Outpost_2_Cycle_Outpost",3);
        AutoTrajectory traj5 = getTrajectory("Outpost_2_Cycle_Outpost",4);
        AutoTrajectory traj6 = getTrajectory("Outpost_2_Cycle_Outpost",5);
        AutoTrajectory traj7 = getTrajectory("Outpost_2_Cycle_Outpost",6);
        AutoTrajectory traj8 = getTrajectory("Outpost_2_Cycle_Outpost",7);
        AutoTrajectory traj9 = getTrajectory("Outpost_2_Cycle_Outpost",8);

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectory(traj1).alongWith(Commands.print("traj1")),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj2).alongWith(Commands.print("traj2")),
                    CatzSuperstructure.Instance.intakeOFF(),
                    followTrajectory(traj3)
                ),
                    CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj4),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBallsNoJiggle(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),

            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj5),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj6),
                    CatzSuperstructure.Instance.intakeOFF(),
                    followTrajectory(traj7)
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj8),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            followTrajectoryWhileShooting(traj9),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT + AutonConstants.PRELOAD_SHOOTING_WAIT + AutonConstants.OUTPOST_SCORING_WAIT),
            Commands.print("done")
        );
    }
}
