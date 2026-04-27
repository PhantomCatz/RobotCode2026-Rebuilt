package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class NZ_Hoard_Outpost_Bump extends AutoRoutineBase {
    public NZ_Hoard_Outpost_Bump(){
        super("NZ_Hoard_Outpost_Bump");

        AutoTrajectory traj1 = getTrajectory("NZ_Hoard_Outpost_Bump",0);
        AutoTrajectory traj2 = getTrajectory("NZ_Hoard_Outpost_Bump",1);
        AutoTrajectory traj3 = getTrajectory("NZ_Hoard_Outpost_Bump",2);

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(2),
                    followTrajectoryWithAccuracy(traj1),
                    CatzSuperstructure.Instance.intakeON()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            CatzSuperstructure.Instance.intakeON(),
            Commands.deadline(
                Commands.sequence(
                    followTrajectoryWithAccuracy(traj2),
                    Commands.waitSeconds(2),
                    followTrajectoryWithAccuracy(traj3)
                ),
                CatzSuperstructure.Instance.cmdHoardStandby()
            ),
            Commands.print("done")
        );
    }
}
