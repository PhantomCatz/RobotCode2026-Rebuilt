package frc.robot.Autonomous.routines;


import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Depot_3_Cycle extends AutoRoutineBase {
    public Depot_3_Cycle(){
        super("Depot_3_Cycle");

        AutoTrajectory traj1 = getTrajectory("Depot_3_Cycle",0);
        AutoTrajectory traj2 = getTrajectory("Depot_3_Cycle",1);
        AutoTrajectory traj3 = getTrajectory("Depot_3_Cycle",2);
        AutoTrajectory traj4 = getTrajectory("Depot_3_Cycle",3);
        AutoTrajectory traj5 = getTrajectory("Depot_3_Cycle",4);
        AutoTrajectory traj6 = getTrajectory("Depot_3_Cycle",5);
        AutoTrajectory traj7 = getTrajectory("Depot_3_Cycle",6);
        AutoTrajectory traj8 = getTrajectory("Depot_3_Cycle",7);
        AutoTrajectory traj9 = getTrajectory("Depot_3_Cycle",8);
        AutoTrajectory traj10 = getTrajectory("Depot_3_Cycle",9);
        AutoTrajectory traj11 = getTrajectory("Depot_3_Cycle",10);
        AutoTrajectory traj12 = getTrajectory("Depot_3_Cycle",11);
        AutoTrajectory traj13 = getTrajectory("Depot_3_Cycle",12);
        AutoTrajectory traj14 = getTrajectory("Depot_3_Cycle",13);
        AutoTrajectory traj15 = getTrajectory("Depot_3_Cycle",14);
        AutoTrajectory traj16 = getTrajectory("Depot_3_Cycle",15);

        // traj2.atTime("Intake2").onTrue(CatzSuperstructure.Instance.intakeON());
        // traj6.atTime("IntakeStop+RampUp6").onTrue(CatzSuperstructure.Instance.intakeOFF());
        // traj8.atTime("Score8").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));

        // traj10.atTime("Intake10").onTrue(CatzSuperstructure.Instance.intakeON());
        // traj11.atTime("IntakeStop+RampUp11").onTrue(CatzSuperstructure.Instance.intakeOFF());
        // traj13.atTime("Score+StowIntake+TrackTower13")

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
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                    CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectory(traj5),
                followTrajectory(traj6),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj7),
                    followTrajectory(traj8),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj9),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj10),
                    followTrajectory(traj11)
                ),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj12),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj13),
                    CatzSuperstructure.Instance.intakeOFF(),
                    followTrajectory(traj14)
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj15),
                    followTrajectory(traj16)
                ),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.print("done")
        );
    }
}
