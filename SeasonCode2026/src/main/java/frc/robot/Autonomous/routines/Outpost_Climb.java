package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Outpost_Climb extends AutoRoutineBase{
    public Outpost_Climb(){
        super("Outpost_Climb");

        AutoTrajectory traj1 = getTrajectory("Outpost_Climb",0);
        AutoTrajectory traj2 = getTrajectory("Outpost_Climb",1);
        AutoTrajectory traj3 = getTrajectory("Outpost_Climb",2);
        AutoTrajectory traj4 = getTrajectory("Outpost_Climb",3);
        AutoTrajectory traj5 = getTrajectory("Outpost_Climb",4);
        AutoTrajectory traj6 = getTrajectory("Outpost_Climb",5);

        traj2.atTime("Intake2").onTrue(CatzSuperstructure.Instance.intakeON());
        traj4.atTime("IntakeStop+RampUp4").onTrue(CatzSuperstructure.Instance.intakeOFF());

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectory(traj1),
                    followTrajectory(traj2)
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            followTrajectory(traj3),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj4),
                    followTrajectory(traj5)
                ),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            followTrajectory(traj6),

            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT)
            .deadlineFor(CatzSuperstructure.Instance.jiggleIntakeCommand()),
            CatzSuperstructure.Instance.intakeOFF().andThen(CatzSuperstructure.Instance.deployIntake()),

            CatzSuperstructure.Instance.autoClimbCommand(),

            Commands.print("Climb"), //TODO
            Commands.print("done")
        );
    }
}
