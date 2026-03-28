package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class TowerSwipe extends AutoRoutineBase{

    private final AutoTrajectory traj1;
    private final AutoTrajectory traj2;
    private final AutoTrajectory traj3;

    public TowerSwipe() {
        super("TowerSwipe");
        traj1 = getTrajectory("TowerSwipe",0);
        traj2 = getTrajectory("TowerSwipe",1);
        traj3 = getTrajectory("TowerSwipe",2);
    }

    public Command getPathCommand() {
        return Commands.deadline(
            Commands.sequence(
                CatzSuperstructure.Instance.deployIntake(),
                Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                followTrajectory(traj1),
                CatzSuperstructure.Instance.intakeON(),
                followTrajectory(traj2),
                CatzSuperstructure.Instance.intakeOFF(),
                followTrajectory(traj3)
            ),
            CatzSuperstructure.Instance.trackTower()
        );
    }
}
