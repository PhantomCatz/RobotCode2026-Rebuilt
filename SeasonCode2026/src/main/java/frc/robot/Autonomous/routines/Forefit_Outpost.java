package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import frc.robot.Autonomous.AutoRoutineBase;

public class Forefit_Outpost extends AutoRoutineBase{
    public Forefit_Outpost(){
        super("Forefit_Outpost");

        AutoTrajectory trajpre = getTrajectory("Forefit_Outpost",0);
        AutoTrajectory traj1 = getTrajectory("Forefit_Outpost",1);
        AutoTrajectory traj2 = getTrajectory("Forefit_Outpost",2);
        AutoTrajectory traj3 = getTrajectory("Forefit_Outpost",3);

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
