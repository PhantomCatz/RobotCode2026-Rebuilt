package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Outpost_2_Cycle_Bump_Elim extends AutoRoutineBase {
    public Outpost_2_Cycle_Bump_Elim(){
        super("Outpost_2_Cycle_Bump_Elim");

        AutoTrajectory traj1 = getTrajectory("Outpost_2_Cycle_Bump_Elim",0);
        AutoTrajectory traj2 = getTrajectory("Outpost_2_Cycle_Bump_Elim",1);
        AutoTrajectory traj3 = getTrajectory("Outpost_2_Cycle_Bump_Elim",2);
        AutoTrajectory traj4 = getTrajectory("Outpost_2_Cycle_Bump_Elim",3);
        AutoTrajectory traj5 = getTrajectory("Outpost_2_Cycle_Bump_Elim",4);
        AutoTrajectory traj6 = getTrajectory("Outpost_2_Cycle_Bump_Elim",5);


        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectory(traj1).alongWith(Commands.print("traj1")),
                    CatzSuperstructure.Instance.intakeON().alongWith(Commands.print("IntakeON")),
                    followTrajectoryWithAccuracy(traj2).alongWith(Commands.print("2"))
                ),
                    CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectory(traj3).alongWith(Commands.print("3")),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            CatzSuperstructure.Instance.intakeOFF().alongWith(Commands.print("IntakeOFF")),
            shootAllBallsNoJiggle(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT+1.2),

            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj4).alongWith(Commands.print("4")),
                    CatzSuperstructure.Instance.intakeON().alongWith(Commands.print("intakeOn")),
                    followTrajectoryWithAccuracy(traj5).alongWith(Commands.print("5"))
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj6).alongWith(Commands.print("6")),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            CatzSuperstructure.Instance.intakeOFF().alongWith(Commands.print("IntakeOFF")),
            shootAllBalls(AutonConstants.OUTPOST_SCORING_WAIT),
            Commands.print("done")
        );
    }
}
