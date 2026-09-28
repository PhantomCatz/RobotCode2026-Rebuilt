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
        AutoTrajectory traj3 = getTrajectory("Outpost_3_Cycle",2);
        AutoTrajectory traj4 = getTrajectory("Outpost_3_Cycle",3);
        AutoTrajectory traj5 = getTrajectory("Outpost_3_Cycle",4);
        AutoTrajectory traj6 = getTrajectory("Outpost_3_Cycle",5);
        AutoTrajectory traj7 = getTrajectory("Outpost_3_Cycle",6);
        AutoTrajectory traj8 = getTrajectory("Outpost_3_Cycle",7);
        AutoTrajectory traj9 = getTrajectory("Outpost_3_Cycle",8);
        AutoTrajectory traj10 = getTrajectory("Outpost_3_Cycle",9);
        AutoTrajectory traj11 = getTrajectory("Outpost_3_Cycle",10);
        AutoTrajectory traj12 = getTrajectory("Outpost_3_Cycle",11);



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
                    followTrajectory(traj3)
                ),
                    CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj4),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBallsNoJiggle(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),

            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj5),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj6),
                    CatzSuperstructure.Instance.intakeOFF(),
                    followTrajectory(traj7)
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj8),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBallsNoJiggle(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),

            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj9),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj10),
                    CatzSuperstructure.Instance.intakeOFF(),
                    followTrajectory(traj11)
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj12),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.print("done")
        );
    }
}
