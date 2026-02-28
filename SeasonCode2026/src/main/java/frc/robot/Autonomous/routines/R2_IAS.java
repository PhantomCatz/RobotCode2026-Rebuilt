package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class R2_IAS extends AutoRoutineBase {
    public R2_IAS(){
        super("R2_IAS");

        AutoTrajectory traj1 = getTrajectory("R2_IAS",0);
        AutoTrajectory traj2 = getTrajectory("R2_IAS",1);
        AutoTrajectory traj3 = getTrajectory("R2_IAS",2);
        AutoTrajectory traj4 = getTrajectory("R2_IAS",3);
        AutoTrajectory traj5 = getTrajectory("R2_IAS",4);
        AutoTrajectory traj6 = getTrajectory("R2_IAS",5);
        AutoTrajectory traj7 = getTrajectory("R2_IAS",6);
        AutoTrajectory traj8 = getTrajectory("R2_IAS",7);
        AutoTrajectory traj9 = getTrajectory("R2_IAS",8);
        AutoTrajectory traj10 = getTrajectory("R2_IAS",9);
        AutoTrajectory traj11 = getTrajectory("R2_IAS",10);
        AutoTrajectory traj12 = getTrajectory("R2_IAS",11);
        AutoTrajectory traj13 = getTrajectory("R2_IAS",12);
        AutoTrajectory traj14 = getTrajectory("R2_IAS",13);

        traj2.atTime("Intake2").onTrue(CatzSuperstructure.Instance.intakeON());
        traj6.atTime("IntakeStop+RampUp6").onTrue(CatzSuperstructure.Instance.cmdHubStandby()
                                                   .alongWith(CatzSuperstructure.Instance.intakeOFF()));
        traj8.atTime("Score8").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));

        traj10.atTime("Intake10").onTrue(CatzSuperstructure.Instance.intakeON());
        traj11.atTime("IntakeStop+RampUp11").onTrue(CatzSuperstructure.Instance.intakeOFF()
                                                   .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        traj13.atTime("Score+StowIntake+TrackTower13").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT)
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
            followTrajectoryWithAccuracy(traj9),
            followTrajectoryWithAccuracy(traj10),
            followTrajectoryWithAccuracy(traj11),
            followTrajectoryWithAccuracy(traj12),
            followTrajectoryWithAccuracy(traj13),
            followTrajectoryWithAccuracy(traj14),
            Commands.print("Climb"),
            Commands.print("done")
        );
    }
}
