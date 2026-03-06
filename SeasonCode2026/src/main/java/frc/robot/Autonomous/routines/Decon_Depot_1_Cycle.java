package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Decon_Depot_1_Cycle extends AutoRoutineBase{
    public Decon_Depot_1_Cycle(){
        super("Decon_Depot_1_Cycle");
        AutoTrajectory traj1 = getTrajectory("Decon_Depot_1_Cycle",0);
        AutoTrajectory traj2 = getTrajectory("Decon_Depot_1_Cycle",1);
        AutoTrajectory traj3 = getTrajectory("Decon_Depot_1_Cycle",2);
        AutoTrajectory traj4 = getTrajectory("Decon_Depot_1_Cycle",3);
        AutoTrajectory traj5 = getTrajectory("Decon_Depot_1_Cycle",4);
        AutoTrajectory traj6 = getTrajectory("Decon_Depot_1_Cycle",5);
        AutoTrajectory traj7 = getTrajectory("Decon_Depot_1_Cycle",6);


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
                    followTrajectory(traj3),
                    followTrajectory(traj4),
                    followTrajectory(traj5)
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            CatzSuperstructure.Instance.intakeOFF(),
            followTrajectory(traj6),
            Commands.deadline(
                followTrajectory(traj7),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            CatzSuperstructure.Instance.autoClimbCommand(),
            Commands.print("Climb"),
            Commands.print("done")
        );
    }
}
