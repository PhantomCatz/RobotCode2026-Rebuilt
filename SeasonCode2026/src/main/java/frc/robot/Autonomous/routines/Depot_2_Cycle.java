package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Depot_2_Cycle extends AutoRoutineBase{
    public Depot_2_Cycle(){
        super("Depot_2_Cycle");
        AutoTrajectory traj1 = getTrajectory("Depot_2_Cycle",0);
        AutoTrajectory traj2 = getTrajectory("Depot_2_Cycle",1);
        AutoTrajectory traj3 = getTrajectory("Depot_2_Cycle",2);
        AutoTrajectory traj4 = getTrajectory("Depot_2_Cycle",3);
        AutoTrajectory traj5 = getTrajectory("Depot_2_Cycle",4);
        AutoTrajectory traj6 = getTrajectory("Depot_2_Cycle",5);


        // traj2.atTime("Intake2").onTrue(CatzSuperstructure.Instance.intakeON());
        // traj6.atTime("RampUp+StopIntake6").onTrue(CatzSuperstructure.Instance.intakeOFF());
        // traj7.atTime("StowIntake+TrackTower7").onTrue();

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectory(traj1),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj2),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj3),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
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
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.print("done")
        );
    }
}
