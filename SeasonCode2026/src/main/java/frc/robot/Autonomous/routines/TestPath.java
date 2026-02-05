package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.Timer;
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
            Commands.print("1: " + Timer.getFPGATimestamp()),
            followTrajectoryWithAccuracy(traj2),
            Commands.print("2: " + Timer.getFPGATimestamp()),
            followTrajectoryWithAccuracy(traj3),
            Commands.print("3: " + Timer.getFPGATimestamp()),
            followTrajectoryWithAccuracy(traj4),
            Commands.print("4: " + Timer.getFPGATimestamp())
        );
    }
}
