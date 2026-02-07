package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous.AutoRoutineBase;

public class Forefit_Depot extends AutoRoutineBase{
    public Forefit_Depot(){
        super("Forefit_Outpost");

        AutoTrajectory trajpre = getTrajectory("Forefit_Depot",0);
        AutoTrajectory traj1 = getTrajectory("Forefit_Depot",1);
        AutoTrajectory traj2 = getTrajectory("Forefit_Depot",2);

        prepRoutine(
            trajpre,
            new SequentialCommandGroup(
                followTrajectoryWithAccuracy(trajpre),
                followTrajectoryWithAccuracy(traj1),
                followTrajectoryWithAccuracy(traj2)
            
            
            )
        );
    }
}
