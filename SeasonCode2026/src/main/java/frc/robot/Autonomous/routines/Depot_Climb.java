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
        AutoTrajectory traj6 = getTrajectory("Depot_Climb",5);
        AutoTrajectory traj7 = getTrajectory("Depot_Climb",6);

        traj2.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT).alongWith(Commands.print("Intake")));
        traj4.atTime("RampUp+StopIntake4").onTrue(CatzSuperstructure.Instance.rampUpFlywheels(RegressionMode.HUB).alongWith(Commands.print("RampUp+StopIntake"))
                                                    .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)));
        traj4.atTime("Score4").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));

        prepRoutine(
            traj1,
            CatzSuperstructure.Instance.toggleIntakeDeploy(),
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            followTrajectoryWithAccuracy(traj5),
            followTrajectoryWithAccuracy(traj6),
            followTrajectoryWithAccuracy(traj7),
            Commands.print("Climb"), //TODO
            Commands.print("done")
        );
    }
}
