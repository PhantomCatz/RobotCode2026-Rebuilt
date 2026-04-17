package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Depot_2_Cycle_Bump extends AutoRoutineBase {
    public Depot_2_Cycle_Bump(){
        super("Depot_2_Cycle_Bump");

        AutoTrajectory traj1 = getTrajectory("Depot_2_Cycle_Bump",0);
        AutoTrajectory traj2 = getTrajectory("Depot_2_Cycle_Bump",1);
        AutoTrajectory traj3 = getTrajectory("Depot_2_Cycle_Bump",2);
        AutoTrajectory traj4 = getTrajectory("Depot_2_Cycle_Bump",3);
        AutoTrajectory traj5 = getTrajectory("Depot_2_Cycle_Bump",4);
        AutoTrajectory traj6 = getTrajectory("Depot_2_Cycle_Bump",5);
        AutoTrajectory traj7 = getTrajectory("Depot_2_Cycle_Bump",6);

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectory(traj1),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectoryWithAccuracy(traj2),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectory(traj3),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBallsNoJiggle(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),

            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj4),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj5)
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            CatzSuperstructure.Instance.intakeOFF(),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj6),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT + AutonConstants.PRELOAD_SHOOTING_WAIT + AutonConstants.OUTPOST_SCORING_WAIT),
            followTrajectoryWithAccuracy(traj7),
            Commands.print("done")
        );
    }
}
