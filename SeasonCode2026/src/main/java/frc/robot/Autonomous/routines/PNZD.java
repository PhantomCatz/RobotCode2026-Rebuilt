package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import frc.robot.Autonomous.AutoRoutineBase;

public class PNZD extends AutoRoutineBase{
    public PNZD(){
        super("PNZD");

        AutoTrajectory traj1 = getTrajectory("PNZD",0);
        AutoTrajectory traj2 = getTrajectory("PNZD",1);
        AutoTrajectory traj3 = getTrajectory("PNZD",2);
        AutoTrajectory traj4 = getTrajectory("PNZD",3);
        AutoTrajectory traj5 = getTrajectory("PNZD",4);

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
