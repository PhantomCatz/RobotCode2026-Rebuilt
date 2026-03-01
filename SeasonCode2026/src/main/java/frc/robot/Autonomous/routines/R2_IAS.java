package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class R2_IAS extends AutoRoutineBase {
    public R2_IAS(){
        super("R2_IAS");

        AutoTrajectory traj1 = getTrajectory("R2_IAS",0);
        AutoTrajectory traj2 = getTrajectory("R2_IAS",1);
        AutoTrajectory traj3 = getTrajectory("R2_IAS",2);
        AutoTrajectory traj4 = getTrajectory("R2_IAS",3);
        AutoTrajectory traj5 = getTrajectory("R2_IAS",4);
        AutoTrajectory traj6 = getTrajectory("R2_IAS",5);
        AutoTrajectory traj7 = getTrajectory("R2_IAS",6);
        AutoTrajectory traj8 = getTrajectory("R2_IAS",7);
        AutoTrajectory traj9 = getTrajectory("R2_IAS",8);
        AutoTrajectory traj10 = getTrajectory("R2_IAS",9);
        AutoTrajectory traj11 = getTrajectory("R2_IAS",10);
        AutoTrajectory traj12 = getTrajectory("R2_IAS",11);
        AutoTrajectory traj13 = getTrajectory("R2_IAS",12);
        AutoTrajectory traj14 = getTrajectory("R2_IAS",13);

        traj2.atTime("Intake2").onTrue(CatzSuperstructure.Instance.intakeON());
        traj6.atTime("IntakeStop+RampUp6").onTrue(CatzSuperstructure.Instance.intakeOFF());
        // traj8.atTime("Score8").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));

        traj10.atTime("Intake10").onTrue(CatzSuperstructure.Instance.intakeON());
        traj11.atTime("IntakeStop+RampUp11").onTrue(CatzSuperstructure.Instance.intakeOFF());
        // traj13.atTime("Score+StowIntake+TrackTower13")

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectory(traj1),
                    followTrajectory(traj2),
                    followTrajectory(traj3),
                    followTrajectory(traj4),
                    followTrajectory(traj5)
                ),
                CatzSuperstructure.Instance.deployIntake()
                    .alongWith(CatzSuperstructure.Instance.trackStaticHub())
            ),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj6),
                    followTrajectory(traj7)
                ), 
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            followTrajectory(traj8),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj9),
                    followTrajectory(traj10),
                    followTrajectory(traj11),
                    followTrajectory(traj12),
                    followTrajectory(traj13)
                ), 
                CatzSuperstructure.Instance.cmdHubStandby()

            ),
            Commands.deadline(
                Commands.sequence(
                    Commands.waitSeconds(5)
                ),
                shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT)
                .alongWith(CatzSuperstructure.Instance.stowIntake())
            ),
            Commands.deadline(
                Commands.sequence(
                    followTrajectoryWithAccuracy(traj14)
                ),
                CatzSuperstructure.Instance.trackTower()
            ),
            Commands.print("Climb"),
            Commands.print("done")
        );
    }
}
