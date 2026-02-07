package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import frc.robot.Autonomous.AutoRoutineBase;

public class R3IAS extends AutoRoutineBase{
    public R3IAS(){
        super("R3IAS");

        AutoTrajectory traj1 = getTrajectory("R3IAS",0);
        AutoTrajectory traj2 = getTrajectory("R3IAS",1);
        AutoTrajectory traj3 = getTrajectory("R3IAS",2);
        AutoTrajectory traj4 = getTrajectory("R3IAS",3);
        AutoTrajectory traj5 = getTrajectory("R3IAS",4);
        AutoTrajectory traj6 = getTrajectory("R3IAS",5);


        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1),
            // CatzSuperstructure.Instance.ScoreFuel(),
            // CatzSuperstructure.Instance.IntakeFuel(),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            // CatzSuperstructure.Instance.ScoreFuel(),
            followTrajectoryWithAccuracy(traj4),
            followTrajectoryWithAccuracy(traj5),
            // CatzSuperstructure.Instance.IntakeFuel(),
            // CatzSuperstructure.Instance.ScoreFuel(),
            followTrajectoryWithAccuracy(traj6)
            // CatzSuperstructure.Instance.Climb()
        );
    }
}
