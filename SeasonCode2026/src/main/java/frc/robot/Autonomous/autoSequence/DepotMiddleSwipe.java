package frc.robot.Autonomous.autoSequence;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class DepotMiddleSwipe extends AutoRoutineBase{

    private final AutoTrajectory traj1;
    // private final AutoTrajectory traj2;

    public DepotMiddleSwipe() {
        super("DepotMiddleSwipe");
        traj1 = getTrajectory("DepotMiddleSwipe",0);
        // traj2 = getTrajectory("DepotSwipe",1);
    }

    public Command getPathCommand() {
        return Commands.deadline(
            Commands.sequence(
                CatzSuperstructure.Instance.deployIntake(),
                CatzSuperstructure.Instance.intakeON(),
                followTrajectory(traj1),
                // followTrajectory(traj2),
                CatzSuperstructure.Instance.intakeOFF()
            ),
            CatzSuperstructure.Instance.trackTower()
        );
    }
}
