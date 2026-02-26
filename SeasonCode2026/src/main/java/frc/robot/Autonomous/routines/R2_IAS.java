package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

public class R2_IAS extends AutoRoutineBase {
    public R2_IAS(){
        super("R2_IAS");

        AutoTrajectory traj1 = getTrajectory("R2_IAS",0);
        AutoTrajectory traj2 = getTrajectory("R2_IAS",1);
        AutoTrajectory traj3 = getTrajectory("R2_IAS",2);
        AutoTrajectory traj4 = getTrajectory("R2_IAS",3);
        AutoTrajectory traj5 = getTrajectory("R2_IAS",4);
        AutoTrajectory traj6 = getTrajectory("R2_IAS",5);

        traj1.atTime("Intake1").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj2.atTime("IntakeStop+RampUp2").onTrue(CatzSuperstructure.Instance.cmdHubStandby()
                                                   .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)));
        traj3.atTime("Intake3").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj4.atTime("IntakeStop+RampUp4").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                                   .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        prepRoutine(
            traj1,
            Commands.runOnce(() -> CommandScheduler.getInstance().schedule(CatzSuperstructure.Instance.deployIntake())),

            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT + AutonConstants.PRELOAD_SHOOTING_WAIT),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            followTrajectoryWithAccuracy(traj5),
            followTrajectoryWithAccuracy(traj6),
            Commands.print("Climb"),
            Commands.print("done")
        );
    }
}
