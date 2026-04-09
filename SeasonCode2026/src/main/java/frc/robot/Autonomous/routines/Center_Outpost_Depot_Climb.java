package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Center_Outpost_Depot_Climb extends AutoRoutineBase {
    public Center_Outpost_Depot_Climb(){
        super("Center_Outpost_Depot_Climb");

        AutoTrajectory traj1 = getTrajectory("Center_Outpost_Depot_Climb",0);
        AutoTrajectory traj2 = getTrajectory("Center_Outpost_Depot_Climb",1);
        AutoTrajectory traj3 = getTrajectory("Center_Outpost_Depot_Climb",2);
        AutoTrajectory traj4 = getTrajectory("Center_Outpost_Depot_Climb",3);


        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    followTrajectory(traj1).alongWith(Commands.print("traj1")),
                    Commands.waitSeconds(1.4), //Waiting for depot fuel to go into hopper
                    followTrajectory(traj2),
                    CatzSuperstructure.Instance.intakeON()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectory(traj3).alongWith(Commands.print("3")),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            CatzSuperstructure.Instance.intakeOFF().alongWith(Commands.print("IntakeOFF")),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT+0.5),
            followTrajectory(traj4),
            CatzSuperstructure.Instance.autoClimbCommand(),
            Commands.print("done")
        );
    }
}
