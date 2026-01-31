package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import frc.robot.Autonomous.AutoRoutineBase;

public class Outpostclimb extends AutoRoutineBase{
    public Outpostclimb(){
        super("Outpostclimb");

        AutoTrajectory traj1 = getTrajectory("Outpostclimb",0);
        AutoTrajectory traj2 = getTrajectory("Outpostclimb",1);
        AutoTrajectory traj3 = getTrajectory("Outpostclimb",2);
        AutoTrajectory traj4 = getTrajectory("Outpostclimb",3);
        AutoTrajectory traj5 = getTrajectory("Outpostclimb",4);
        AutoTrajectory traj6 = getTrajectory("Outpostclimb",5);

        prepRoutine(
            traj1,
            // CatzSuperstructure.Instance.ScoreFuel(),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            // CatzSuperstructure.Instance.IntakeFuel(),
            followTrajectoryWithAccuracy(traj4),
            // CatzSuperstructure.Instance.ScoreFuel(),
            followTrajectoryWithAccuracy(traj5),
            followTrajectoryWithAccuracy(traj6)
            // CatzSuperstructure.Instance.Climb()
        );
    }
}
