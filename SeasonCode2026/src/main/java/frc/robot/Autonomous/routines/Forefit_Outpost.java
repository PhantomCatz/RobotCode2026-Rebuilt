package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;


public class Forefit_Outpost extends AutoRoutineBase{
    public Forefit_Outpost(){
        super("Forefit_Outpost");

        AutoTrajectory traj1 = getTrajectory("Forefit_Outpost",0);
        AutoTrajectory traj2 = getTrajectory("Forefit_Outpost",1);
        AutoTrajectory traj3 = getTrajectory("Forefit_Outpost",2);
        AutoTrajectory traj4 = getTrajectory("Forefit_Outpost",3);
        AutoTrajectory traj5 = getTrajectory("Forefit_Outpost",4);
        AutoTrajectory traj6 = getTrajectory("Forefit_Outpost",5);
        AutoTrajectory traj7 = getTrajectory("Forefit_Outpost",6);

        traj2.atTime("Intake+RampUp2").onTrue(CatzSuperstructure.Instance.intakeON());
        // traj3.atTime("Hoard3").onTrue();

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectoryWithAccuracy(traj1),
                    followTrajectoryWithAccuracy(traj2)
                ),
                CatzSuperstructure.Instance.deployIntake()
                    .alongWith(CatzSuperstructure.Instance.cmdHoardStandby())
            ),
            Commands.deadline(
                Commands.sequence(
                    followTrajectoryWithAccuracy(traj3),
                    followTrajectoryWithAccuracy(traj4),
                    followTrajectoryWithAccuracy(traj5),
                    followTrajectoryWithAccuracy(traj6),
                    followTrajectoryWithAccuracy(traj7)
                ), 
                CatzSuperstructure.Instance.cmdHoardShoot()
            ),
            Commands.deadline(
                Commands.waitSeconds(3.1),
                CatzSuperstructure.Instance.cmdShooterStop()
                .alongWith(CatzSuperstructure.Instance.intakeOFF())
                .alongWith(CatzSuperstructure.Instance.stowIntake())
            ),
            
            Commands.print("done")
        );
    }
}
