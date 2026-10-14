package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import org.wpilib.command2.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class DepotSwiped extends AutoRoutineBase {
    public DepotSwiped(){
        super("DepotSwiped");

        AutoTrajectory traj1 = getTrajectory("DepotSwiped",0);
        AutoTrajectory traj2 = getTrajectory("DepotSwiped",1);
        AutoTrajectory traj3 = getTrajectory("DepotSwiped",2);
        AutoTrajectory traj4 = getTrajectory("DepotSwiped",3);
        // AutoTrajectory traj7 = getTrajectory("DepotSwiped",6);


        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    // Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectoryWithAccuracy(traj1).alongWith(Commands.print("traj1")),
                    Commands.waitSeconds(2.5).alongWith(Commands.print("waiting")),
                    CatzSuperstructure.Instance.intakeON().alongWith(Commands.print("intakeon")),
                    followTrajectoryWithAccuracy(traj2).alongWith(Commands.print("traj2")),
                    CatzSuperstructure.Instance.intakeOFF().alongWith(Commands.print("intakeoff"))
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj3).alongWith(Commands.print("traj3")),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT+0.5).alongWith(Commands.print("shooting")),
            followTrajectoryWithAccuracy(traj4).alongWith(Commands.print("traj4")),
            Commands.print("done")
        );
    }
}
