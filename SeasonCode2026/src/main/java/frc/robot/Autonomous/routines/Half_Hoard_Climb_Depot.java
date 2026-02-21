package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression.RegressionMode;

public class Half_Hoard_Climb_Depot extends AutoRoutineBase{
    public Half_Hoard_Climb_Depot(){
        super("Half_Hoard_Climb_Depot");

        AutoTrajectory traj1 = getTrajectory("Half_Hoard_Climb_Depot",0);
        AutoTrajectory traj2 = getTrajectory("Half_Hoard_Climb_Depot",1);
        AutoTrajectory traj3 = getTrajectory("Half_Hoard_Climb_Depot",2);
        AutoTrajectory traj4 = getTrajectory("Half_Hoard_Climb_Depot",3);
        AutoTrajectory traj5 = getTrajectory("Half_Hoard_Climb_Depot",4);
        AutoTrajectory traj6 = getTrajectory("Half_Hoard_Climb_Depot",5);
        AutoTrajectory traj7 = getTrajectory("Half_Hoard_Climb_Depot",6);
        AutoTrajectory traj8 = getTrajectory("Half_Hoard_Climb_Depot",7);
        AutoTrajectory traj9 = getTrajectory("Half_Hoard_Climb_Depot",8);

        traj1.atTime("RampUp+Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT)
                                         .alongWith(CatzSuperstructure.Instance.cmdHoardStandby()));
        traj5.atTime("HoardStop6").onTrue(CatzSuperstructure.Instance.cmdShooterStop());
        traj7.atTime("RampUp+IntakeStop8").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                                    .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));

        prepRoutine(
            traj1,
            CatzSuperstructure.Instance.deployIntake(),
            shootAllBalls(AutonConstants.PRELOAD_SHOOTING_WAIT),

            followTrajectoryWithAccuracy(traj1),

            CatzSuperstructure.Instance.cmdHoardShoot(),

            followTrajectoryWithAccuracy(traj2),

            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),

            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            followTrajectoryWithAccuracy(traj5),
            followTrajectoryWithAccuracy(traj6),
            followTrajectoryWithAccuracy(traj7),

            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),

            followTrajectoryWithAccuracy(traj8),
            followTrajectoryWithAccuracy(traj9),
            Commands.print("Climb"), //TODO
            Commands.print("done")
        );
    }
}
