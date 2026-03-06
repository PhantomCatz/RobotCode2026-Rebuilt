package frc.robot.Autonomous.routines;


import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Outpost_3_Cycle extends AutoRoutineBase {
    public Outpost_3_Cycle(){
        super("Outpost_3_Cycle");

        AutoTrajectory traj1 = getTrajectory("Outpost_3_Cycle",0);
        AutoTrajectory traj2 = getTrajectory("Outpost_3_Cycle",1);
        AutoTrajectory traj5 = getTrajectory("Outpost_3_Cycle",4);
        AutoTrajectory traj7 = getTrajectory("Outpost_3_Cycle",6);
        AutoTrajectory traj9 = getTrajectory("Outpost_3_Cycle",8);
        AutoTrajectory traj11 = getTrajectory("Outpost_3_Cycle",10);
        AutoTrajectory traj12 = getTrajectory("Outpost_3_Cycle",11);
        AutoTrajectory traj13 = getTrajectory("Outpost_3_Cycle",12);
        AutoTrajectory traj14 = getTrajectory("Outpost_3_Cycle",13);
        AutoTrajectory traj16 = getTrajectory("Outpost_3_Cycle",15);

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
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                    CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectory(traj5),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj7),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj9),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                Commands.sequence(
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
                followTrajectory(traj16),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.print("done")
        );
    }
}
