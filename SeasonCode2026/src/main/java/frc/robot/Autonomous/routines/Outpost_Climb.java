package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression.RegressionMode;

public class Outpost_Climb extends AutoRoutineBase{
    public Outpost_Climb(){
        super("Outpost_Climb");
//Warning if this crashes its jadens fault
        AutoTrajectory traj1 = getTrajectory("Outpost_Climb",0);
        AutoTrajectory traj2 = getTrajectory("Outpost_Climb",1);
        AutoTrajectory traj3 = getTrajectory("Outpost_Climb",2);
        AutoTrajectory traj4 = getTrajectory("Outpost_Climb",3);

        traj1.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.MAX_SPEED));
        traj2.atTime("RampUp+IntakeStop2").onTrue(CatzSuperstructure.Instance.trackTargetAndRampUp(RegressionMode.HUB)
                                                    .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)));

        prepRoutine(
            traj1,
            shootAllBalls(AutonConstants.PRELOAD_SHOOTING_WAIT),
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            Commands.print("Climb5"), //TODO
            Commands.print("done")
        );
    }
}
