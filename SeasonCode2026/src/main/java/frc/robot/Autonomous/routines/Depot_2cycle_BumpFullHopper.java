package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Depot_2cycle_BumpFullHopper extends AutoRoutineBase {
    public Depot_2cycle_BumpFullHopper(){
        super("Depot_2cycle_BumpFullHopper");

        AutoTrajectory traj1 = getTrajectory("Depot_2cycle_BumpFullHopper",0);
        AutoTrajectory traj2 = getTrajectory("Depot_2cycle_BumpFullHopper",1);
        AutoTrajectory traj3 = getTrajectory("Depot_2cycle_BumpFullHopper",2);
        AutoTrajectory traj4 = getTrajectory("Depot_2cycle_BumpFullHopper",3);
        AutoTrajectory traj5 = getTrajectory("Depot_2cycle_BumpFullHopper",4);
        AutoTrajectory traj6 = getTrajectory("Depot_2cycle_BumpFullHopper",5);
        AutoTrajectory traj7 = getTrajectory("Depot_2cycle_BumpFullHopper",6);
        AutoTrajectory traj8 = getTrajectory("Depot_2cycle_BumpFullHopper",7);
        AutoTrajectory traj9 = getTrajectory("Depot_2cycle_BumpFullHopper",8);
        AutoTrajectory traj10 = getTrajectory("Depot_2cycle_BumpFullHopper",9);

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj1),
                    CatzSuperstructure.Instance.deployIntake(),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj2),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectory(traj3),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootWhileMove(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),

            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj4),
                    CatzSuperstructure.Instance.cmdShooterStop(),
                    Commands.waitSeconds(3),
                    shootAllBallsNoJiggle(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
                    followTrajectory(traj5),
                    CatzSuperstructure.Instance.intakeON()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                Commands.sequence(
                    followTrajectoryWithAccuracy(traj6),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            followTrajectory(traj7),
            shootWhileMove(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT + AutonConstants.PRELOAD_SHOOTING_WAIT + AutonConstants.OUTPOST_SCORING_WAIT),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj8),
                    CatzSuperstructure.Instance.cmdShooterStop(),
                    Commands.waitSeconds(3),
                    shootAllBallsNoJiggle(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
                    followTrajectory(traj9),
                    CatzSuperstructure.Instance.intakeON()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            followTrajectory(traj10),
            CatzSuperstructure.Instance.intakeOFF(),
            Commands.print("done")
        );
    }
}
