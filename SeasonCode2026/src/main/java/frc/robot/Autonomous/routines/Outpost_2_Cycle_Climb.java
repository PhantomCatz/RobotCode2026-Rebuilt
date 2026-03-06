package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Outpost_2_Cycle_Climb extends AutoRoutineBase {
    public Outpost_2_Cycle_Climb(){
        super("Outpost_2_Cycle_Climb");

        AutoTrajectory traj1 = getTrajectory("Outpost_2_Cycle_Climb",0);
        AutoTrajectory traj2 = getTrajectory("Outpost_2_Cycle_Climb",1);
        AutoTrajectory traj3 = getTrajectory("Outpost_2_Cycle_Climb",2);
        AutoTrajectory traj4 = getTrajectory("Outpost_2_Cycle_Climb",3);
        AutoTrajectory traj5 = getTrajectory("Outpost_2_Cycle_Climb",4);
        AutoTrajectory traj6 = getTrajectory("Outpost_2_Cycle_Climb",5);
        AutoTrajectory traj7 = getTrajectory("Outpost_2_Cycle_Climb",6);
        AutoTrajectory traj8 = getTrajectory("Outpost_2_Cycle_Climb",7);
        AutoTrajectory traj9 = getTrajectory("Outpost_2_Cycle_Climb",8);
        AutoTrajectory traj10 = getTrajectory("Outpost_2_Cycle_Climb",9);
        AutoTrajectory traj11 = getTrajectory("Outpost_2_Cycle_Climb",10);

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
                    followTrajectory(traj3),
                    followTrajectory(traj4),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                    CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectory(traj5),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            followTrajectory(traj6),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            followTrajectory(traj7),
            followTrajectory(traj8),
            CatzSuperstructure.Instance.intakeON(),
            followTrajectory(traj9),
            CatzSuperstructure.Instance.intakeOFF(),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj10),
                    followTrajectory(traj11)
                ),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            // CatzSuperstructure.Instance.autoClimbCommand(),
            Commands.print("Climb"),
            Commands.print("done")
        );
    }
}
