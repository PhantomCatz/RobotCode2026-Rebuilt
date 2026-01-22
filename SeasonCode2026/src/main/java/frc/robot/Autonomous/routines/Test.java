package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Test extends AutoRoutineBase{
    public Test(){
        super("Test");

        AutoTrajectory traj1 = getTrajectory("TestPath",0);
        AutoTrajectory traj2 = getTrajectory("TestPath1",1);

        prepRoutine(
            traj1,
            CatzSuperstructure.Instance.flywheelManualCommand(),
            new ParallelCommandGroup(),
            followTrajectoryWithAccuracy(traj2)
        );
    }
}
