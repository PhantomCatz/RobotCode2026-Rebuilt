package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Better_Outpost_2_Cycle_Outpost extends AutoRoutineBase {
    public Better_Outpost_2_Cycle_Outpost(){
        super("Better_Outpost_2_Cycle_Outpost");

        AutoTrajectory traj1 = getTrajectory("Better_Outpost_2_Cycle_Outpost",0);
        AutoTrajectory traj2 = getTrajectory("Better_Outpost_2_Cycle_Outpost",1);
        AutoTrajectory traj3 = getTrajectory("Better_Outpost_2_Cycle_Outpost",2);
        AutoTrajectory traj4 = getTrajectory("Better_Outpost_2_Cycle_Outpost",3);
        AutoTrajectory traj5 = getTrajectory("Better_Outpost_2_Cycle_Outpost",4);
        AutoTrajectory traj6 = getTrajectory("Better_Outpost_2_Cycle_Outpost",5);
        AutoTrajectory traj7 = getTrajectory("Better_Outpost_2_Cycle_Outpost",6);
        AutoTrajectory traj8 = getTrajectory("Better_Outpost_2_Cycle_Outpost",7);
        AutoTrajectory traj9 = getTrajectory("Better_Outpost_2_Cycle_Outpost",8);

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectory(traj1),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj2),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj3),
                    followTrajectory(traj4)
                ),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBallsNoJiggle(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),

            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj5),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj6),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj7),
                    followTrajectory(traj8)
                ),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            followTrajectoryWhileShooting(traj9),
            shootAllBalls(AutonConstants.RETURN_TIME_BUFFER),
            Commands.print("done")
        );
    }
}
