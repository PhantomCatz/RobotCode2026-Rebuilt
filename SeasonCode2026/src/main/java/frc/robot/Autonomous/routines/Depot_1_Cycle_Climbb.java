package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Depot_1_Cycle_Climbb extends AutoRoutineBase{
    public Depot_1_Cycle_Climbb(){
        super("Depot_1_Cycle_Climbb");
        AutoTrajectory traj1 = getTrajectory("Depot_1_Cycle_Climbb",0);
        AutoTrajectory traj2 = getTrajectory("Depot_1_Cycle_Climbb",1);
        AutoTrajectory traj3 = getTrajectory("Depot_1_Cycle_Climbb",2);
        AutoTrajectory traj4 = getTrajectory("Depot_1_Cycle_Climbb",3);


        // traj2.atTime("Intake2").onTrue(CatzSuperstructure.Instance.intakeON());
        // traj6.atTime("RampUp+StopIntake6").onTrue(CatzSuperstructure.Instance.intakeOFF());
        // traj7.atTime("StowIntake+TrackTower7").onTrue();

/*
 * Note: this path is a troll path made by the strategy lead ;-;
 */

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectory(traj1),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj2),
                    Commands.waitSeconds(1.5),
                    followTrajectory(traj3),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj4),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(13.0),
            Commands.print("done")
        );
    }
}
