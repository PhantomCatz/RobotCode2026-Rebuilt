package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous.AutoRoutineBase;

public class Outpostclimb extends AutoRoutineBase{
    public Outpostclimb(){
        super("Outpostclimb");
//Warning if this crashes its jadens fault
        AutoTrajectory traj1 = getTrajectory("Outpostclimb",0);
        AutoTrajectory traj2 = getTrajectory("Outpostclimb",1);
        AutoTrajectory traj3 = getTrajectory("Outpostclimb",2);
        AutoTrajectory traj4 = getTrajectory("Outpostclimb",3);

        traj1.atTime("Score1").onTrue(Commands.print("Score1"));
        traj1.atTime("Intake2").onTrue(Commands.print("Intake2"));
        traj3.atTime("Score3").onTrue(Commands.print("Score3"));

        prepRoutine(
            traj1,
            new SequentialCommandGroup(
                followTrajectoryWithAccuracy(traj1),
                followTrajectoryWithAccuracy(traj2),
                followTrajectoryWithAccuracy(traj3),
                followTrajectoryWithAccuracy(traj4),
                Commands.print("Climb5")
            )
        );
    }
}
