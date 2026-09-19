package frc.robot.Autonomous.routines;


import choreo.auto.AutoTrajectory;
import org.wpilib.command2.Commands;
import frc.robot.Autonomous.AutoRoutineBase;

public class NewPath extends AutoRoutineBase{
    public NewPath(){
        super("NewPath");
            //i hate pid
        AutoTrajectory traj1 = getTrajectory("NewPath");

        prepRoutine(
            traj1,
            Commands.sequence(followTrajectory(traj1))
        );
    }
}
