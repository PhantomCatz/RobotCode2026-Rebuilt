package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression.RegressionMode;

public class Depot_Climb extends AutoRoutineBase{
    public Depot_Climb(){
        super("Depot_Climb");

        AutoTrajectory traj1 = getTrajectory("Depot_Climb",0);
        AutoTrajectory traj2 = getTrajectory("Depot_Climb",1);
        AutoTrajectory traj3 = getTrajectory("Depot_Climb",2);
        AutoTrajectory traj4 = getTrajectory("Depot_Climb",3);
        AutoTrajectory traj5 = getTrajectory("Depot_Climb",4);

        traj1.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj3.atTime("RampUp+StopIntake3").onTrue(CatzSuperstructure.Instance.trackTargetAndRampUp(RegressionMode.HUB)
                                                    .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)));

        prepRoutine(
            traj1,
            CatzSuperstructure.Instance.toggleIntakeDeploy(),
            shootAllBalls(AutonConstants.PRELOAD_SHOOTING_WAIT),

            followTrajectoryWithAccuracy(traj1),

            followTrajectoryWithAccuracy(traj2),

            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),

            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            followTrajectoryWithAccuracy(traj5),
            Commands.print("Climb"), //TODO
            Commands.print("done")
        );
    }
}
