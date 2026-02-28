package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

public class R1_IAS extends AutoRoutineBase{
    public R1_IAS(){
        super("R1_IAS");

        AutoTrajectory traj1 = getTrajectory("R1_IAS",0);
        AutoTrajectory traj2 = getTrajectory("R1_IAS",1);
        AutoTrajectory traj3 = getTrajectory("R1_IAS",2);
        AutoTrajectory traj4 = getTrajectory("R1_IAS",3);
        AutoTrajectory traj5 = getTrajectory("R1_IAS",4);
        AutoTrajectory traj6 = getTrajectory("R1_IAS",5);
        AutoTrajectory traj7 = getTrajectory("R1_IAS",6);
        AutoTrajectory traj8 = getTrajectory("R1_IAS",7);
        AutoTrajectory traj9 = getTrajectory("R1_IAS",8);
        AutoTrajectory traj10 = getTrajectory("R1_IAS",9);
        AutoTrajectory traj11 = getTrajectory("R1_IAS",10);
        AutoTrajectory traj12 = getTrajectory("R1_IAS",11);

        traj2.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj5.atTime("StopIntake+RampUp5").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                                    .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        traj6.atTime("Score6").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));
        traj8.atTime("Intake8").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj9.atTime("StopIntake+RampUp9").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                                   .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        traj11.atTime("Score+StowIntake+TrackTower11").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT)
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
            Commands.print("Climb"),
            Commands.print("done")
        );
    }
}
