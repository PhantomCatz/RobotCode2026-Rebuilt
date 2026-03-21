package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;


public class Forefit_Outpost extends AutoRoutineBase{
    public Forefit_Outpost(){
        super("Forefit_Outpost");

        AutoTrajectory traj1 = getTrajectory("Forefit_Outpost",0);
        AutoTrajectory traj2 = getTrajectory("Forefit_Outpost",1);

        // traj2.atTime("Intake+RampUp2").onTrue(CatzSuperstructure.Instance.intakeON());
        // traj3.atTime("Hoard3").onTrue();

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectory(traj1),
                    CatzSuperstructure.Instance.intakeON()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectory(traj2),
                CatzSuperstructure.Instance.toggleCmdHoardShoot()
            ),
            CatzSuperstructure.Instance.cmdShooterStop()
                .alongWith(CatzSuperstructure.Instance.intakeOFF()),

            Commands.print("done")
        );
    }
}
