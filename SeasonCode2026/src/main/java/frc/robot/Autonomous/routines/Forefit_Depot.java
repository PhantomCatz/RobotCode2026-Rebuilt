package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import frc.robot.Autonomous.AutoRoutineBase;

public class Forefit_Depot extends AutoRoutineBase{
    public Forefit_Depot(){
        super("Forefit_Outpost");

        AutoTrajectory trajpre = getTrajectory("Forefit_Depot",0);
        AutoTrajectory traj1 = getTrajectory("Forefit_Depot",1);
        AutoTrajectory traj2 = getTrajectory("Forefit_Depot",2);
        AutoTrajectory traj3 = getTrajectory("Forefit_Depot",3);

        prepRoutine(
            trajpre,
            // CatzSuperstructure.Instance.ScoreFuel(),
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            // CatzSuperstructure.Instance.AutonStartHoard(),
            followTrajectoryWithAccuracy(traj3)
        );
    }
}
