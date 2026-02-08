package frc.robot.Autonomous.routines;


import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous.AutoRoutineBase;

public class Test extends AutoRoutineBase{
    public Test(){
        super("Test");

        AutoTrajectory traj1 = getTrajectory("Testy",0);
        AutoTrajectory traj2 = getTrajectory("Testy",1);

        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1),
            new WaitCommand(1.0),
            followTrajectoryWithAccuracy(traj2)
        );
    }
}
