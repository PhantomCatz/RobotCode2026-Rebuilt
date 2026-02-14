package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression.RegressionMode;

public class Half_Hoard_Climb_Outpost extends AutoRoutineBase{
    public Half_Hoard_Climb_Outpost(){
        super("Half_Hoard_Climb_Outpost");

        AutoTrajectory traj1 = getTrajectory("Half_Hoard_Climb_Outpost",0);
        AutoTrajectory traj2 = getTrajectory("Half_Hoard_Climb_Outpost",1);
        AutoTrajectory traj3 = getTrajectory("Half_Hoard_Climb_Outpost",2);
        AutoTrajectory traj4 = getTrajectory("Half_Hoard_Climb_Outpost",3);
        AutoTrajectory traj5 = getTrajectory("Half_Hoard_Climb_Outpost",4);
        AutoTrajectory traj6 = getTrajectory("Half_Hoard_Climb_Outpost",5);
        AutoTrajectory traj7 = getTrajectory("Half_Hoard_Climb_Outpost",6);
        AutoTrajectory traj8 = getTrajectory("Half_Hoard_Climb_Outpost",7);


        traj1.atTime("Intake+RampUp2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.MAX_SPEED)
                                         .alongWith(CatzSuperstructure.Instance.trackTargetAndRampUp(RegressionMode.CLOSE_HOARD)));
        traj1.atTime("Hoard").onTrue(CatzSuperstructure.Instance.cmdHoardShoot());
        traj4.atTime("HoardStop5").onTrue(CatzSuperstructure.Instance.cmdFullStop());
        traj5.atTime("RampUp+Intake6").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.MAX_SPEED)
                                         .alongWith(CatzSuperstructure.Instance.trackTargetAndRampUp(RegressionMode.HUB)));
        prepRoutine(
            traj1,

            shootAllBalls(AutonConstants.PRELOAD_SHOOTING_WAIT),

            followTrajectoryWithAccuracy(traj1),

            CatzSuperstructure.Instance.cmdHoardShoot(),

            followTrajectoryWithAccuracy(traj2),

            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),

            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            followTrajectoryWithAccuracy(traj5),
            followTrajectoryWithAccuracy(traj6),

            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            followTrajectoryWithAccuracy(traj7),

            followTrajectoryWithAccuracy(traj8),
            Commands.print("Climb"), //TODO
            Commands.print("done")
        );
    }
}
