package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
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

        traj1.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj2.atTime("IntakeStop+RampUp1").onTrue(CatzSuperstructure.Instance.cmdHubStandby()
                                                   .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT))
                                                   .alongWith(Commands.print("stopping intake and ramping")));
        traj3.atTime("Intake4").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj4.atTime("IntakeStop+RampUp2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                                   .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        prepRoutine(
            traj1,
            // CatzSuperstructure.Instance.deployIntake(),
            followTrajectoryWithAccuracy(traj1),
            Commands.print("finish1"),
            followTrajectory(traj2),
            Commands.print("finish2"),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT + AutonConstants.PRELOAD_SHOOTING_WAIT),
            Commands.print("finishshoot"),
            followTrajectoryWithAccuracy(traj3),
            Commands.print("finish3"),
            followTrajectory(traj4),
            Commands.print("finish4"),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.print("finishshoot"),
            followTrajectoryWithAccuracy(traj5),
            Commands.print("finish5"),
            followTrajectoryWithAccuracy(traj6),
            Commands.print("finish6"),
            Commands.print("Climb"),
            Commands.print("done")
        );
    }
}
