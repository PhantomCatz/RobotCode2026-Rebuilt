package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;

public class TestPath extends AutoRoutineBase{
    public TestPath(){
        super("TestPath");

        AutoTrajectory traj1 = getTrajectory("TestPath",0);
        AutoTrajectory traj2 = getTrajectory("TestPath",1);
        AutoTrajectory traj3 = getTrajectory("TestPath",2);
        AutoTrajectory traj4 = getTrajectory("TestPath",3);


        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1),
            Commands.print("Done With path 1, Waiting"),
            // Commands.waitSeconds(2),
            Commands.print("Running traj 2"),
            followTrajectoryWithAccuracy(traj2),
            Commands.print("Done With path 2, Waiting"),
            // Commands.waitSeconds(2),
            Commands.print("Running traj 3"),
            followTrajectoryWithAccuracy(traj3),
            Commands.print("Done With path 3, Waiting"),
            // Commands.waitSeconds(2),
            Commands.print("Running traj 4"),
            followTrajectoryWithAccuracy(traj4),
            Commands.print("Finished")
        );
    }
}
