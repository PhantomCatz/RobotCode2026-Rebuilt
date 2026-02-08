package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

public class R2IAS extends AutoRoutineBase{
    public R2IAS(){
        super("R2IAS");

        AutoTrajectory traj1 = getTrajectory("R2IASOut");
        AutoTrajectory traj2 = getTrajectory("R2IASIn");

        prepRoutine(
            traj1,
            CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.H_SETPOINT),
            followTrajectoryWithAccuracy(traj1),
            CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT),
            followTrajectoryWithAccuracy(traj2)
        );
    }
}
