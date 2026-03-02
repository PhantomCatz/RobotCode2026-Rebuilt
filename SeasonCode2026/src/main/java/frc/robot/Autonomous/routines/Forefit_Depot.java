package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Forefit_Depot extends AutoRoutineBase{
    public Forefit_Depot(){
        super("Forefit_Depot");

        AutoTrajectory traj1 = getTrajectory("Forefit_Depot",0);
        AutoTrajectory traj2 = getTrajectory("Forefit_Depot",1);
        AutoTrajectory traj3 = getTrajectory("Forefit_Depot",2);
        AutoTrajectory traj4 = getTrajectory("Forefit_Depot",3);
        AutoTrajectory traj5 = getTrajectory("Forefit_Depot",4);
        AutoTrajectory traj6 = getTrajectory("Forefit_Depot",5);
        AutoTrajectory traj7 = getTrajectory("Forefit_Depot",6);

        // traj2.atTime("Intake+RampUp2").onTrue(CatzSuperstructure.Instance.intakeON());
        // traj3.atTime("Hoard3").onTrue();

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectory(traj1),
                    followTrajectory(traj2)
                ),
                CatzSuperstructure.Instance.cmdHoardStandby()
                    .alongWith(CatzSuperstructure.Instance.deployIntake())
                    .alongWith(Commands.waitUntil(traj2.atTime("Intake+RampUp2"))
                                   .andThen(CatzSuperstructure.Instance.intakeON()))
            ),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj3),
                    followTrajectory(traj4),
                    followTrajectory(traj5),
                    followTrajectory(traj6),
                    followTrajectory(traj7)
                ),
                CatzSuperstructure.Instance.cmdHoardShoot()
            ),
            Commands.deadline(
                Commands.waitSeconds(4.4),
                CatzSuperstructure.Instance.stowIntake()
                    .alongWith(CatzSuperstructure.Instance.cmdShooterStop()
                    .alongWith(CatzSuperstructure.Instance.intakeOFF()))
            ),
            Commands.print("done")
        );
    }
}
