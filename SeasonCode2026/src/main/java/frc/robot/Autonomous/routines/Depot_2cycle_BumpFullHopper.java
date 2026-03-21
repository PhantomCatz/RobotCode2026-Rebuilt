package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Depot_2cycle_BumpFullHopper extends AutoRoutineBase{
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


        // traj2.atTime("Intake2").onTrue(CatzSuperstructure.Instance.intakeON());
        // traj6.atTime("RampUp+StopIntake6").onTrue(CatzSuperstructure.Instance.intakeOFF());
        // traj7.atTime("StowIntake+TrackTower7").onTrue();

        prepRoutine(
            traj1,

            CatzSuperstructure.Instance.intakeON(),
            followTrajectory(traj1),
            CatzSuperstructure.Instance.intakeOFF(),
            followTrajectory(traj2),


            Commands.deadline(
                Commands.sequence(
                    //shootWhileMove(3),
                    followTrajectory(traj3)
                ),
                Commands.parallel(
                    CatzSuperstructure.Instance.trackStaticHub(),
                    CatzSuperstructure.Instance.cmdHubStandby()
                )
            ),

            followTrajectory(traj4),
            shootAllBalls(3),
            Commands.waitSeconds(3),

            CatzSuperstructure.Instance.intakeON(),
            followTrajectory(traj5),
            CatzSuperstructure.Instance.intakeOFF(),
            followTrajectory(traj6),

            Commands.deadline(
                Commands.sequence(
                    //shootWhileMove(3),
                    followTrajectory(traj7)
                ),
                Commands.parallel(
                    CatzSuperstructure.Instance.trackStaticHub(),
                    CatzSuperstructure.Instance.cmdHubStandby()
                )
            ),

            Commands.waitSeconds(3),
            followTrajectory(traj8),

            CatzSuperstructure.Instance.intakeON(),
            followTrajectory(traj9),
            CatzSuperstructure.Instance.intakeOFF(),
            followTrajectory(traj10),


            /*
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    followTrajectory(traj1),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj2),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                CatzSuperstructure.Instance.cmdHubStandby()
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
                    followTrajectory(traj5),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj6),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT + AutonConstants.PRELOAD_SHOOTING_WAIT),
            */
            Commands.print("done")
        );
    }
}
