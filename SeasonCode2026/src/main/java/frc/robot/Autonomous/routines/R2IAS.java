package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.CatzSubsystems.CatzShooter.AimCalculations;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;

public class R2IAS extends AutoRoutineBase{
    public R2IAS(){
        super("R2IAS");

        AutoTrajectory traj1 = getTrajectory("R2IAS", 0);
        AutoTrajectory traj2 = getTrajectory("R2IAS", 1);
        AutoTrajectory traj3 = getTrajectory("R2IAS", 2);
        AutoTrajectory traj4 = getTrajectory("R2IAS", 3);
        AutoTrajectory traj5 = getTrajectory("R2IAS", 4);

        prepRoutine(
            traj1,
            CatzTurret.Instance.followSetpointCommand(() -> AimCalculations.calculateHubTrackingSetpoint()),
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            followTrajectoryWithAccuracy(traj5)
        );
    }
}
