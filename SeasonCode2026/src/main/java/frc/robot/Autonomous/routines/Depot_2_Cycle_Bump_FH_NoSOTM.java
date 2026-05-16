package frc.robot.Autonomous.routines;


import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Depot_2_Cycle_Bump_FH_NoSOTM extends AutoRoutineBase {
    public Depot_2_Cycle_Bump_FH_NoSOTM(){
        super("Depot_2_Cycle_Bump_FH_NoSOTM");

        AutoTrajectory traj1 = getTrajectory("Depot_2_Cycle_Bump_FH_NoSOTM",0);
        AutoTrajectory traj2 = getTrajectory("Depot_2_Cycle_Bump_FH_NoSOTM",1);
        AutoTrajectory traj3 = getTrajectory("Depot_2_Cycle_Bump_FH_NoSOTM",2);
        AutoTrajectory traj4 = getTrajectory("Depot_2_Cycle_Bump_FH_NoSOTM",3);
        AutoTrajectory traj5 = getTrajectory("Depot_2_Cycle_Bump_FH_NoSOTM",4);
        AutoTrajectory traj6 = getTrajectory("Depot_2_Cycle_Bump_FH_NoSOTM",5);
        AutoTrajectory traj7 = getTrajectory("Depot_2_Cycle_Bump_FH_NoSOTM",6);
        AutoTrajectory traj8 = getTrajectory("Depot_2_Cycle_Bump_FH_NoSOTM",7);
        AutoTrajectory traj9 = getTrajectory("Depot_2_Cycle_Bump_FH_NoSOTM",8);
        AutoTrajectory traj10 = getTrajectory("Depot_2_Cycle_Bump_FH_NoSOTM",9);

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectory(traj1).alongWith(Commands.print("traj1")),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj2).alongWith(Commands.print("traj2")),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj3).alongWith(Commands.print("traj3")),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj4),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj5).alongWith(Commands.print("traj5"))
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj6).alongWith(Commands.print("traj6")),
                    CatzSuperstructure.Instance.intakeOFF(),
                    followTrajectoryWithAccuracy(traj7).alongWith(Commands.print("traj7"))
                ),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj8),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj9).alongWith(Commands.print("traj9"))
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            followTrajectory(traj10).alongWith(Commands.print("traj10")),
            CatzSuperstructure.Instance.intakeOFF(),
            Commands.print("done")
        );
    }
}
