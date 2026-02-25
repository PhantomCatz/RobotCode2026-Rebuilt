package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
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

        traj2.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT)
                                         .alongWith(CatzSuperstructure.Instance.deployIntake()));
        traj3.atTime("StopIntake+RampUp3").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                                    .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        traj6.atTime("Intake6").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj7.atTime("StopIntake+RampUp7").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                                   .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));

        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            followTrajectoryWithAccuracy(traj5),
            followTrajectoryWithAccuracy(traj6),
            followTrajectoryWithAccuracy(traj7),
            followTrajectoryWithAccuracy(traj8),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            followTrajectoryWithAccuracy(traj9),
            followTrajectoryWithAccuracy(traj10),
            Commands.print("Climb"),
            Commands.print("done")
        );
    }
}
