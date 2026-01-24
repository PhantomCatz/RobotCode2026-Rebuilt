package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import frc.robot.Autonomous.AutoRoutineBase;

public class DepotClimb extends AutoRoutineBase{
    public DepotClimb(){
        super("DepotClimb");

        AutoTrajectory trajpre = getTrajectory("DepotClimb",0);
        AutoTrajectory traj1 = getTrajectory("DepotClimb",1);
        AutoTrajectory traj2 = getTrajectory("DepotClimb",2);
        AutoTrajectory traj3 = getTrajectory("DepotClimb",3);
        AutoTrajectory traj4 = getTrajectory("DepotClimb",4);
        AutoTrajectory traj5 = getTrajectory("DepotClimb",5);

        prepRoutine(
            trajpre,
            followTrajectoryWithAccuracy(trajpre),
            // CatzSuperstructure.Instance.ScoreFuel(),
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            // CatzSuperstructure.Instance.IntakeFuel(),
            followTrajectoryWithAccuracy(traj3),
            // CatzSuperstructure.Instance.ScoreFuel(),
            followTrajectoryWithAccuracy(traj4),
            followTrajectoryWithAccuracy(traj5)
            // CatzSuperstructure.Instance.Climb(),

        );
    }
}
