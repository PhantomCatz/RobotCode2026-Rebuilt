package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Outpost_Climb extends AutoRoutineBase{
    public Outpost_Climb(){
        super("Outpost_Climb");

        AutoTrajectory traj1 = getTrajectory("Outpost_Climb",0);
        AutoTrajectory traj2 = getTrajectory("Outpost_Climb",1);
        AutoTrajectory traj3 = getTrajectory("Outpost_Climb",2);
        AutoTrajectory traj4 = getTrajectory("Outpost_Climb",3);
        AutoTrajectory traj5 = getTrajectory("Outpost_Climb",4);
        AutoTrajectory traj6 = getTrajectory("Outpost_Climb",5);
        AutoTrajectory traj7 = getTrajectory("Outpost_Climb",6);
        AutoTrajectory traj8 = getTrajectory("Outpost_Climb",7);

        traj2.atTime("Intake2").onTrue(CatzSuperstructure.Instance.intakeON());
        traj4.atTime("IntakeStop+RampUp4").onTrue(CatzSuperstructure.Instance.cmdHubStandby()
                                                    .alongWith(CatzSuperstructure.Instance.intakeOFF()));
        traj7.atTime("Score+StowIntake+TrackTower7").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT)
                                                    .alongWith(CatzSuperstructure.Instance.stowIntake())
                                                    .alongWith(CatzSuperstructure.Instance.trackTower()));
        prepRoutine(
            traj1,
            Commands.runOnce(() -> CommandScheduler.getInstance().schedule(CatzSuperstructure.Instance.deployIntake().alongWith(CatzSuperstructure.Instance.trackStaticHub()))),
            Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            followTrajectoryWithAccuracy(traj5),
            followTrajectoryWithAccuracy(traj6),
            followTrajectoryWithAccuracy(traj7),
            followTrajectoryWithAccuracy(traj8),
            Commands.print("Climb"), //TODO
            Commands.print("done")
        );
    }
}
