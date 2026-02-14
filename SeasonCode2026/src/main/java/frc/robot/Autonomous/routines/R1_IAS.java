package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression.RegressionMode;

public class R1_IAS extends AutoRoutineBase{
    public R1_IAS(){
        super("R1_IAS");

        AutoTrajectory traj1 = getTrajectory("R1_IAS",0);
        AutoTrajectory traj2 = getTrajectory("R1_IAS",1);
        AutoTrajectory traj3 = getTrajectory("R1_IAS",2);
        AutoTrajectory traj4 = getTrajectory("R1_IAS",3);
        AutoTrajectory traj5 = getTrajectory("R1_IAS",4);
        AutoTrajectory traj6 = getTrajectory("R1_IAS",5);

        traj1.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj1.atTime("StopIntake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT));
        traj2.atTime("RampUp3").onTrue(CatzSuperstructure.Instance.trackTargetAndRampUp(RegressionMode.HUB));
        traj3.atTime("Intake4").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj3.atTime("StopIntake+RampUp").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                                   .alongWith(CatzSuperstructure.Instance.trackTargetAndRampUp(RegressionMode.HUB)));

        prepRoutine(
            traj1,
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
