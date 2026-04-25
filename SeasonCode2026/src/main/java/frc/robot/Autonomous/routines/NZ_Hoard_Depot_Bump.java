package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class NZ_Hoard_Depot_Bump extends AutoRoutineBase {
    public NZ_Hoard_Depot_Bump(){
        super("NZ_Hoard_Depot_Bump");

        AutoTrajectory traj1 = getTrajectory("NZ_Hoard_Depot_Bump",0);
        AutoTrajectory traj2 = getTrajectory("NZ_Hoard_Depot_Bump",1);
        AutoTrajectory traj3 = getTrajectory("NZ_Hoard_Depot_Bump",2);

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
            Commands.deadline(
                followTrajectory(traj2),
                CatzSuperstructure.Instance.cmdHoardStandby()
            ),
            shootAllBallsNoJiggle(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),

            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj3),
                    CatzSuperstructure.Instance.intakeON()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            CatzSuperstructure.Instance.cmdHoardShoot(),
            Commands.print("done")
        );
    }
}
