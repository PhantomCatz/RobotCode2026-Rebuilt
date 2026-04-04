package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Outpost_2_Cycle_Bump extends AutoRoutineBase {
    public Outpost_2_Cycle_Bump(){
        super("Outpost_2_Cycle_Bump");

        AutoTrajectory traj1 = getTrajectory("Outpost_2_Cycle_Bump",0);
        AutoTrajectory traj2 = getTrajectory("Outpost_2_Cycle_Bump",1);
        AutoTrajectory traj3 = getTrajectory("Outpost_2_Cycle_Bump",2);
        AutoTrajectory traj4 = getTrajectory("Outpost_2_Cycle_Bump",3);
        AutoTrajectory traj5 = getTrajectory("Outpost_2_Cycle_Bump",4);
        AutoTrajectory traj6 = getTrajectory("Outpost_2_Cycle_Bump",5);
        AutoTrajectory traj7 = getTrajectory("Outpost_2_Cycle_Bump",6);


        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectory(traj1).alongWith(Commands.print("traj1")),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectoryWithAccuracy(traj2)
                ),
                    CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectory(traj3),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            CatzSuperstructure.Instance.intakeOFF(),
            shootAllBallsNoJiggle(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),

            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj4),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj5)
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj6),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            CatzSuperstructure.Instance.intakeOFF(),
            followTrajectoryWhileShooting(traj7),
            shootAllBallsNoJiggle(AutonConstants.OUTPOST_SCORING_WAIT),
            Commands.print("done")
        );
    }
}
