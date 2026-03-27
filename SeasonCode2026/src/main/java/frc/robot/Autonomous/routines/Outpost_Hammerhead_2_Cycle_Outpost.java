package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Outpost_Hammerhead_2_Cycle_Outpost extends AutoRoutineBase {
    public Outpost_Hammerhead_2_Cycle_Outpost(){
        super("Outpost_Hammerhead_2_Cycle_Outpost");

        AutoTrajectory traj1 = getTrajectory("Outpost_Hammerhead_2_Cycle_Outpost",0);
        AutoTrajectory traj2 = getTrajectory("Outpost_Hammerhead_2_Cycle_Outpost",1);
        AutoTrajectory traj3 = getTrajectory("Outpost_Hammerhead_2_Cycle_Outpost",2);
        AutoTrajectory traj4 = getTrajectory("Outpost_Hammerhead_2_Cycle_Outpost",3);
        AutoTrajectory traj5 = getTrajectory("Outpost_Hammerhead_2_Cycle_Outpost",4);
        AutoTrajectory traj6 = getTrajectory("Outpost_Hammerhead_2_Cycle_Outpost",5);
        AutoTrajectory traj7 = getTrajectory("Outpost_Hammerhead_2_Cycle_Outpost",6);

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
                    followTrajectory(traj1).alongWith(Commands.print("traj1")),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj2).alongWith(Commands.print("traj2")),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                    CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj3).alongWith(Commands.print("traj3")),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBallsNoJiggle(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),

            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj4).alongWith(Commands.print("traj4")),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj5).alongWith(Commands.print("traj5")),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                Commands.sequence(
                    followTrajectoryWithAccuracy(traj6).alongWith(Commands.print("traj6"))
                ),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            // shootWhileMove(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT + AutonConstants.PRELOAD_SHOOTING_WAIT + AutonConstants.OUTPOST_SCORING_WAIT),
            Commands.print("done")
        );
    }
}
