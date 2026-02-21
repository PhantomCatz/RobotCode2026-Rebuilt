package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression.RegressionMode;

public class Half_Hoard_Cycle_Outpost extends AutoRoutineBase{
    public Half_Hoard_Cycle_Outpost(){
        super("Half_Hoard_Cycle_Outpost");

        AutoTrajectory traj1 = getTrajectory("Half_Hoard_Cycle_Outpost",0);
        AutoTrajectory traj2 = getTrajectory("Half_Hoard_Cycle_Outpost",1);
        AutoTrajectory traj3 = getTrajectory("Half_Hoard_Cycle_Outpost",2);
        AutoTrajectory traj4 = getTrajectory("Half_Hoard_Cycle_Outpost",3);
        AutoTrajectory traj5 = getTrajectory("Half_Hoard_Cycle_Outpost",4);
        AutoTrajectory traj6 = getTrajectory("Half_Hoard_Cycle_Outpost",5);
        AutoTrajectory traj7 = getTrajectory("Half_Hoard_Cycle_Outpost",6);
        AutoTrajectory traj8 = getTrajectory("Half_Hoard_Cycle_Outpost",7);
        AutoTrajectory traj9 = getTrajectory("Half_Hoard_Cycle_Outpost",8);
        AutoTrajectory traj10 = getTrajectory("Half_Hoard_Cycle_Outpost",9);
        AutoTrajectory traj11 = getTrajectory("Half_Hoard_Cycle_Outpost",10);
        AutoTrajectory traj12 = getTrajectory("Half_Hoard_Cycle_Outpost",11);


        traj2.atTime("Intake+RampUp2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT)
                                         .alongWith(CatzSuperstructure.Instance.cmdHoardStandby()));
        traj3.atTime("Hoard3").onTrue(CatzSuperstructure.Instance.cmdHoardShoot());
        traj5.atTime("HoardStop5").onTrue(CatzSuperstructure.Instance.cmdShooterStop());
        traj7.atTime("RampUp+IntakeStop7").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                         .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        traj9.atTime("Score9").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));
        traj10.atTime("Intake10").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj12.atTime("RampUp+IntakeStop12").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                         .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
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
            shootAllBalls(AutonConstants.PRELOAD_SHOOTING_WAIT),
            Commands.print("done")
        );
    }
}
