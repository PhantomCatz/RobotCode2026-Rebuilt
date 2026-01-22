package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class R2IAS extends AutoRoutineBase{
    public R2IAS(){
        super("R2IAS");

        AutoTrajectory traj1 = getTrajectory("R2IAS",0);
        AutoTrajectory traj3 = getTrajectory("R2IAS",1);
        AutoTrajectory traj4 = getTrajectory("R2IAS",2);
        AutoTrajectory traj5 = getTrajectory("R2IAS",3);
        AutoTrajectory traj6 = getTrajectory("R2IAS",4);


        prepRoutine(
            traj1,
            // CatzSuperstructure.Instance.IntakeFuel(),
            followTrajectoryWithAccuracy(traj3),
            // CatzSuperstructure.Instance.ScoreFuel(),
            followTrajectoryWithAccuracy(traj4),
            // CatzSuperstructure.Instance.IntakeFuel(),  
            followTrajectoryWithAccuracy(traj5),
            // CatzSuperstructure.Instance.ScoreFuel(),
            followTrajectoryWithAccuracy(traj6)
            // CatzSuperstructure.Instance.IntakeFuel(),
            // CatzSuperstructure.Instance.Hoard(),
        );
    }
}
