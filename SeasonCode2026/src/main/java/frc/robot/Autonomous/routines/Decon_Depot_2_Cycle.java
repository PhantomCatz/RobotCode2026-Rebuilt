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
        AutoTrajectory traj3 = getTrajectory("Decon_Depot_2_Cycle",2);
        AutoTrajectory traj4 = getTrajectory("Decon_Depot_2_Cycle",3);
        AutoTrajectory traj5 = getTrajectory("Decon_Depot_2_Cycle",4);
        AutoTrajectory traj6 = getTrajectory("Decon_Depot_2_Cycle",5);
        AutoTrajectory traj7 = getTrajectory("Decon_Depot_2_Cycle",6);
        AutoTrajectory traj8 = getTrajectory("Decon_Depot_2_Cycle",7);
        AutoTrajectory traj9 = getTrajectory("Decon_Depot_2_Cycle",8);
        AutoTrajectory traj10 = getTrajectory("Decon_Depot_2_Cycle",9);
        AutoTrajectory traj11 = getTrajectory("Decon_Depot_2_Cycle",10);

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
                    followTrajectory(traj3),
                    followTrajectory(traj4)
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            CatzSuperstructure.Instance.intakeOFF(),
            Commands.deadline(
                followTrajectory(traj5),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            followTrajectory(traj6),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj7),
                    followTrajectory(traj8),
                    CatzSuperstructure.Instance.intakeON()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            CatzSuperstructure.Instance.intakeOFF(),
            followTrajectory(traj9),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj10),
                    followTrajectory(traj11)
                ),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            CatzSuperstructure.Instance.stowIntake(),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            CatzSuperstructure.Instance.autoClimbCommand(),
            Commands.print("done")
        );
    }
}
