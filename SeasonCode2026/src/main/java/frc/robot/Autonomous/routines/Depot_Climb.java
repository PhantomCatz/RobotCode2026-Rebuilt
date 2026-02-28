package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Depot_Climb extends AutoRoutineBase{
    public Depot_Climb(){
        super("Depot_Climb");
        AutoTrajectory traj1 = getTrajectory("Depot_Climb",0);
        AutoTrajectory traj2 = getTrajectory("Depot_Climb",1);
        AutoTrajectory traj3 = getTrajectory("Depot_Climb",2);
        AutoTrajectory traj4 = getTrajectory("Depot_Climb",3);
        AutoTrajectory traj5 = getTrajectory("Depot_Climb",4);
        AutoTrajectory traj6 = getTrajectory("Depot_Climb",5);
        AutoTrajectory traj7 = getTrajectory("Depot_Climb",6);
        AutoTrajectory traj8 = getTrajectory("Depot_Climb",7);
        AutoTrajectory traj9 = getTrajectory("Depot_Climb",8);

        traj2.atTime("Intake2").onTrue(CatzSuperstructure.Instance.intakeON());
        traj6.atTime("RampUp+StopIntake6").onTrue(CatzSuperstructure.Instance.intakeOFF());
        // traj7.atTime("StowIntake+TrackTower7").onTrue();



        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectoryWithAccuracy(traj1),
                    followTrajectoryWithAccuracy(traj2),
                    followTrajectoryWithAccuracy(traj3),
                    followTrajectoryWithAccuracy(traj4),
                    followTrajectoryWithAccuracy(traj5)
                ),
                CatzSuperstructure.Instance.deployIntake().alongWith(CatzSuperstructure.Instance.trackStaticHub())
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj6), 
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.deadline(
                Commands.sequence(
                    followTrajectoryWithAccuracy(traj7),
                    followTrajectoryWithAccuracy(traj8),
                    followTrajectoryWithAccuracy(traj9)
                ),
                CatzSuperstructure.Instance.trackTower()
                .alongWith(CatzSuperstructure.Instance.stowIntake())
            ),
            Commands.print("Climb"),
            Commands.print("done")
        );
    }
}
