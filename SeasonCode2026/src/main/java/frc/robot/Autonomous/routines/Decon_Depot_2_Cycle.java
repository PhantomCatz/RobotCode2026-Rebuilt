package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Decon_Depot_2_Cycle extends AutoRoutineBase{
    public Decon_Depot_2_Cycle(){
        super("Decon_Depot_2_Cycle");

        AutoTrajectory traj1 = getTrajectory("Decon_Depot_2_Cycle",0);
        AutoTrajectory traj2 = getTrajectory("Decon_Depot_2_Cycle",1);
        AutoTrajectory traj5 = getTrajectory("Decon_Depot_2_Cycle",2);
        AutoTrajectory traj6 = getTrajectory("Decon_Depot_2_Cycle",3);
        AutoTrajectory traj7 = getTrajectory("Decon_Depot_2_Cycle",4);
        AutoTrajectory traj9 = getTrajectory("Decon_Depot_2_Cycle",5);
        AutoTrajectory traj10 = getTrajectory("Decon_Depot_2_Cycle",6);
        AutoTrajectory traj11 = getTrajectory("Decon_Depot_2_Cycle",7);

        // traj1.atTime("Intake1").onTrue();
        // traj5.atTime("StopIntake+RampUp5").onTrue(
                                                    // .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        // traj6.atTime("Score6").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));
        // traj8.atTime("Intake8").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        // traj9.atTime("StopIntake+RampUp9").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
        //                                            .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        // traj11.atTime("Score+StowIntake+TrackTower11").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT)
        //                                              .alongWith(CatzSuperstructure.Instance.stowIntake())
        //                                              .alongWith(CatzSuperstructure.Instance.trackTower()));

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
                    followTrajectory(traj9),
                    CatzSuperstructure.Instance.intakeOFF(),
                    followTrajectory(traj10)
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj11),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT + AutonConstants.PRELOAD_SHOOTING_WAIT),
            // CatzSuperstructure.Instance.autoClimbCommand(),
            Commands.print("done")
        );
    }
}
