package frc.robot.Autonomous.routines;


import choreo.auto.AutoTrajectory;
import frc.robot.Autonomous.AutoRoutineBase;

public class Test extends AutoRoutineBase{
    public Test(){
        super("Test");

        AutoTrajectory traj1 = getTrajectory("Testy",0);
        AutoTrajectory traj2 = getTrajectory("Testy",1);

        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2)
        );
    }
}
