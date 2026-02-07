package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;

public class Test extends AutoRoutineBase{
    public Test(){
        super("Test");

        AutoTrajectory traj1 = getTrajectory("TestPath");
        traj1.atTime("TestMarker").onTrue(Commands.print("hello world!!!!!"));

        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1)
        );
    }
}
