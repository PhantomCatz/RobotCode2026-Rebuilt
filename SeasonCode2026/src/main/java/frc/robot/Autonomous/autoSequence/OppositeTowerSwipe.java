package frc.robot.Autonomous.autoSequence;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class OppositeTowerSwipe extends AutoRoutineBase{

    private final AutoTrajectory traj1;

    public OppositeTowerSwipe() {
        super("OppositeTowerSwipe");
        traj1 = getTrajectory("OppositeTowerSwipe",0);
    }

    public Command getPathCommand() {
        return Commands.deadline(
            Commands.sequence(
                CatzSuperstructure.Instance.deployIntake(),
                CatzSuperstructure.Instance.intakeON(),
                followTrajectory(traj1),
                CatzSuperstructure.Instance.intakeOFF()
            ),
            CatzSuperstructure.Instance.trackTower()
        );
    }
}
