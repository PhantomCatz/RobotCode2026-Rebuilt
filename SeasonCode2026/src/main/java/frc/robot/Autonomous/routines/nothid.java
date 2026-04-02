package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class nothid extends AutoRoutineBase {
    public nothid() {
        super("nothid");

        AutoTrajectory traj1 = getTrajectory("nothid", 0);
        AutoTrajectory traj2 = getTrajectory("nothid", 1);
        AutoTrajectory traj3 = getTrajectory("nothid", 2);

        prepRoutine(
                traj1,
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.print("intake deployed"),
                    followTrajectory(traj1),
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    Commands.print("intake on"),
                    Commands.print("finsished traj1"),
                    CatzSuperstructure.Instance.intakeON()),

                Commands.sequence(
                    followTrajectory(traj2),
                    CatzSuperstructure.Instance.stowIntake(),
                    Commands.print("intake stowed"),
                    Commands.print("finsished traj2"),
                    CatzSuperstructure.Instance.intakeOFF()),

                Commands.sequence(
                    followTrajectory(traj3),
                    CatzSuperstructure.Instance.cmdHubShoot(),
                    Commands.print("shoot"),
                    Commands.print("finsished traj3")),

        CatzSuperstructure.Instance.cmdShooterStop(),
        Commands.print("finsished traj3"));
    }
}
