package frc.robot.Autonomous.autoSequence;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class OppositeDepotMiddleSwipe extends AutoRoutineBase{

    private final AutoTrajectory traj1;
    // private final AutoTrajectory traj2;

    public OppositeDepotMiddleSwipe() {
        super("OppositeDepotMiddleSwipe");
        traj1 = getTrajectory("OppositeDepotMiddleSwipe",0);
        // traj2 = getTrajectory("DepotSwipe",1);
    }

    public Command getPathCommand() {
        return Commands.sequence(
            CatzSuperstructure.Instance.deployIntake(),
            CatzSuperstructure.Instance.intakeON(),
            followTrajectory(traj1),
            CatzSuperstructure.Instance.intakeOFF()
        );
    }
}
