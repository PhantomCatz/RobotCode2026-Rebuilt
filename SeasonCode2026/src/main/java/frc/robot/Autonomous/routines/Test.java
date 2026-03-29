package frc.robot.Autonomous.routines;


import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Test extends AutoRoutineBase{
    public Test(){
        super("Test");

        AutoTrajectory traj1 = getTrajectory("SOTMTest",0);

        prepRoutine(
            traj1,
            CatzSuperstructure.Instance.deployIntake(),
            followTrajectoryWhileShooting(traj1),
            Commands.print("Done")
        );
    }
}
