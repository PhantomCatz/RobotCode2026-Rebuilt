package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression.RegressionMode;

public class Forefit_Depot extends AutoRoutineBase{
    public Forefit_Depot(){
        super("Forefit_Outpost");

        AutoTrajectory traj1 = getTrajectory("Forefit_Depot",0);
        AutoTrajectory traj2 = getTrajectory("Forefit_Depot",1);
        AutoTrajectory traj3 = getTrajectory("Forefit_Depot",2);
        AutoTrajectory traj4 = getTrajectory("Forefit_Depot",3);
        AutoTrajectory traj5 = getTrajectory("Forefit_Depot",4);
        AutoTrajectory traj6 = getTrajectory("Forefit_Depot",5);

        traj1.atTime("Intake+RampUp1").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT)
                                      .alongWith(CatzSuperstructure.Instance.toggleIntakeDeploy())
                                      .alongWith(CatzSuperstructure.Instance.rampUpFlywheels(RegressionMode.CLOSE_HOARD)));
        traj2.atTime("Hoard2").onTrue(CatzSuperstructure.Instance.cmdHoardShoot());

        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            followTrajectoryWithAccuracy(traj5),
            followTrajectoryWithAccuracy(traj6),
            CatzSuperstructure.Instance.cmdFullStop(),
            Commands.print("done")
        );
    }
}
