package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression.RegressionMode;

public class PNZO extends AutoRoutineBase{
    public PNZO(){
        super("PNZO");

        AutoTrajectory traj1 = getTrajectory("PNZO",0);
        AutoTrajectory traj2 = getTrajectory("PNZO",1);
        AutoTrajectory traj3 = getTrajectory("PNZO",2);
        AutoTrajectory traj4 = getTrajectory("PNZO",3);
        AutoTrajectory traj5 = getTrajectory("PNZO",4);
        AutoTrajectory traj6 = getTrajectory("PNZO",5);
        traj2.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj4.atTime("RampUp+StopIntake4").onTrue(CatzSuperstructure.Instance.rampUpFlywheels(RegressionMode.HUB)
                                                    .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)));
        traj5.atTime("Score5").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));
        traj5.atTime("RampUp+Intake5").onTrue(CatzSuperstructure.Instance.rampUpFlywheels(RegressionMode.HUB)
                                               .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT)));

        prepRoutine(
            traj1,
            CatzSuperstructure.Instance.toggleIntakeDeploy(),
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.print("done")

        );
    }
}
