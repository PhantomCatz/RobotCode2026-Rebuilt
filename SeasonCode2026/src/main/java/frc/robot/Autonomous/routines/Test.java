package frc.robot.Autonomous.routines;


import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous.AutoRoutineBase;

public class Test extends AutoRoutineBase{
    public Test(){
        super("Test");

        AutoTrajectory traj1 = getTrajectory("BumpPathTesting",0);

        prepRoutine(
            traj1,
            followTrajectory(traj1),
            Commands.print("Done")
        );
    }
}
