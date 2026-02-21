package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression.RegressionMode;

public class Half_Hoard_Cycle_Depot extends AutoRoutineBase{
    public Half_Hoard_Cycle_Depot(){
        super("Half_Hoard_Cycle_Depot");

        AutoTrajectory traj1 = getTrajectory("Half_Hoard_Cycle_Depot",0);
        AutoTrajectory traj2 = getTrajectory("Half_Hoard_Cycle_Depot",1);
        AutoTrajectory traj3 = getTrajectory("Half_Hoard_Cycle_Depot",2);
        AutoTrajectory traj4 = getTrajectory("Half_Hoard_Cycle_Depot",3);
        AutoTrajectory traj5 = getTrajectory("Half_Hoard_Cycle_Depot",4);
        AutoTrajectory traj6 = getTrajectory("Half_Hoard_Cycle_Depot",5);
        AutoTrajectory traj7 = getTrajectory("Half_Hoard_Cycle_Depot",6);
        AutoTrajectory traj8 = getTrajectory("Half_Hoard_Cycle_Depot",7);
        AutoTrajectory traj9 = getTrajectory("Half_Hoard_Cycle_Depot",8);
        AutoTrajectory traj10 = getTrajectory("Half_Hoard_Cycle_Depot",9);
        AutoTrajectory traj11 = getTrajectory("Half_Hoard_Cycle_Depot",10);
        AutoTrajectory traj12 = getTrajectory("Half_Hoard_Cycle_Depot",11);
        AutoTrajectory traj13 = getTrajectory("Half_Hoard_Cycle_Depot",12);

        traj2.atTime("RampUp+Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT)
                                         .alongWith(CatzSuperstructure.Instance.rampUpFlywheels(RegressionMode.CLOSE_HOARD)));
        traj3.atTime("Hoard3").onTrue(CatzSuperstructure.Instance.cmdHoardShoot());
        traj6.atTime("HoardStop6").onTrue(CatzSuperstructure.Instance.cmdFullStop());
        traj8.atTime("IntakeStop+RampUp9").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                                    .alongWith(CatzSuperstructure.Instance.rampUpFlywheels(RegressionMode.HUB)));
        traj10.atTime("Score10").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));
        traj11.atTime("Intake11").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj13.atTime("RampUp+IntakeStop13").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                                    .alongWith(CatzSuperstructure.Instance.rampUpFlywheels(RegressionMode.HUB)));
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
            followTrajectoryWithAccuracy(traj8),
            followTrajectoryWithAccuracy(traj9),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.print("done")
        );
    }
}
