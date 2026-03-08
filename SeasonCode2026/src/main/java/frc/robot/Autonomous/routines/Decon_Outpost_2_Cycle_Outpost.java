package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Decon_Outpost_2_Cycle_Outpost extends AutoRoutineBase {
    public Decon_Outpost_2_Cycle_Outpost(){
        super("Decon_Outpost_2_Cycle_Outpost");

        AutoTrajectory traj1 = getTrajectory("Decon_Outpost_2_Cycle_Outpost",0);
        AutoTrajectory traj2 = getTrajectory("Decon_Outpost_2_Cycle_Outpost",1);
        AutoTrajectory traj5 = getTrajectory("Decon_Outpost_2_Cycle_Outpost",2);
        AutoTrajectory traj6 = getTrajectory("Decon_Outpost_2_Cycle_Outpost",3);
        AutoTrajectory traj7 = getTrajectory("Decon_Outpost_2_Cycle_Outpost",4);
        AutoTrajectory traj8 = getTrajectory("Decon_Outpost_2_Cycle_Outpost",5);
        AutoTrajectory traj9 = getTrajectory("Decon_Outpost_2_Cycle_Outpost",6);
        AutoTrajectory traj10 = getTrajectory("Decon_Outpost_2_Cycle_Outpost",7);
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
                    CatzSuperstructure.Instance.intakeOFF(),
                    followTrajectory(traj5)
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj6),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBallsNoJiggle(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),

            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj7),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj8),
                    CatzSuperstructure.Instance.intakeOFF(),
                    followTrajectory(traj9)
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                Commands.sequence(
                    followTrajectoryWithAccuracy(traj10)
                ),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT + AutonConstants.PRELOAD_SHOOTING_WAIT + AutonConstants.OUTPOST_SCORING_WAIT),
            Commands.print("done")
        );
    }
}
