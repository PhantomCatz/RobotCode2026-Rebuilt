package frc.robot.Autonomous.routines;


import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;

public class Test extends AutoRoutineBase{
    public Test(){
        super("Test");

        AutoTrajectory traj1 = getTrajectory("TestPath",0);
        AutoTrajectory traj2 = getTrajectory("TestPath",1);


        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2)
        );
    }
}
