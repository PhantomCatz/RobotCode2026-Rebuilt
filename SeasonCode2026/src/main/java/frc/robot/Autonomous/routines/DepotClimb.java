package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous.AutoRoutineBase;

public class DepotClimb extends AutoRoutineBase{
    public DepotClimb(){
        super("DepotClimb");
        AutoTrajectory traj1 = getTrajectory("DepotClimb",0);
        AutoTrajectory traj2 = getTrajectory("DepotClimb",1);
        AutoTrajectory traj3 = getTrajectory("DepotClimb",2);
        AutoTrajectory traj4 = getTrajectory("DepotClimb",3);
        traj1.atTime("Score1").onTrue(Commands.print("yo gurt"));
        traj2.atTime("Intake2").onTrue(Commands.print("yo gurt"));
        traj3.atTime("Score3").onTrue(Commands.print("yo gurt"));
        traj4.atTime("Climb5").onTrue(Commands.print("Yo gurt"));
        prepRoutine(
            traj1,
            new SequentialCommandGroup(
                followTrajectoryWithAccuracy(traj1),
                followTrajectoryWithAccuracy(traj2),
                followTrajectoryWithAccuracy(traj3),
                followTrajectoryWithAccuracy(traj4)

            ),
            Commands.print("done")
        );
    }
}
