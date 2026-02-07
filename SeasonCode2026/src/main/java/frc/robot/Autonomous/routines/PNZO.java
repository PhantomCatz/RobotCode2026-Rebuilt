package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import frc.robot.Autonomous.AutoRoutineBase;

public class PNZO extends AutoRoutineBase{
    public PNZO(){
        super("PNZO");

        AutoTrajectory traj1 = getTrajectory("PNZO",0);
        AutoTrajectory traj2 = getTrajectory("PNZO",1);
        AutoTrajectory traj3 = getTrajectory("PNZO",2);
        AutoTrajectory traj4 = getTrajectory("PNZO",3);
        AutoTrajectory traj5 = getTrajectory("PNZO",4);

        prepRoutine(
            traj1,
            // CatzSuperstructure.Instance.ScoreFuel(),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            // CatzSuperstructure.Instance.IntakeFuel(),
            followTrajectoryWithAccuracy(traj4),
            // CatzSuperstructure.Instance.ScoreFuel(),
            followTrajectoryWithAccuracy(traj5)
            // CatzSuperstructure.Instance.IntakeFuel(),
            // CatzSuperstructure.Instance.ScoreFuel(),
        );
    }
}
