package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class NZ_Hoard_Outpost_Bump extends AutoRoutineBase {
    public NZ_Hoard_Outpost_Bump(){
        super("NZ_Hoard_Outpost_Bump");

        AutoTrajectory traj1 = getTrajectory("NZ_Hoard_Outpost_Bump",0);
        AutoTrajectory traj2 = getTrajectory("NZ_Hoard_Outpost_Bump",1);
        AutoTrajectory traj3 = getTrajectory("NZ_Hoard_Outpost_Bump",2);
        AutoTrajectory traj4 = getTrajectory("NZ_Hoard_Outpost_Bump",3);

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(1),
                    followTrajectoryWithAccuracy(traj1)
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            CatzSuperstructure.Instance.intakeON(),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj2),
                CatzSuperstructure.Instance.cmdHoardStandby()
            ),
            Commands.deadline(
                Commands.waitSeconds(3),
                CatzSuperstructure.Instance.cmdHoardShoot()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj3),
                CatzSuperstructure.Instance.cmdHoardStandby()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj4),
                CatzSuperstructure.Instance.cmdHoardShoot()
            ),
            Commands.print("done")
        );
    }
}
