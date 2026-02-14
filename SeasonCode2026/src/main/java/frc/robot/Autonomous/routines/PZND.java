package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression.RegressionMode;

public class PZND extends AutoRoutineBase{
    public PZND(){
        super("PZND");

        AutoTrajectory traj1 = getTrajectory("PZND",0);
        AutoTrajectory traj2 = getTrajectory("PZND",1);
        AutoTrajectory traj3 = getTrajectory("PZND",2);

        traj1.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.MAX_SPEED));
        traj3.atTime("RampUp+StopIntake3").onTrue(CatzSuperstructure.Instance.trackTargetAndRampUp(RegressionMode.HUB)
                                                    .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)));
        traj3.atTime("RampUp+Intake").onTrue(CatzSuperstructure.Instance.trackTargetAndRampUp(RegressionMode.HUB)
                                               .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.MAX_SPEED)));

        prepRoutine(
            traj1,
            shootAllBalls(AutonConstants.PRELOAD_SHOOTING_WAIT),
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            followTrajectoryWithAccuracy(traj3),
            CatzSuperstructure.Instance.cmdHubShoot(),
            Commands.print("done")
        );
    }
}
