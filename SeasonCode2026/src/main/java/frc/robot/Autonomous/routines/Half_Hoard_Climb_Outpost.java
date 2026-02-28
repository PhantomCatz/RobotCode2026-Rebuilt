package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

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
        AutoTrajectory traj9 = getTrajectory("Half_Hoard_Climb_Outpost",8);
        AutoTrajectory traj10 = getTrajectory("Half_Hoard_Climb_Outpost",9);

        traj2.atTime("Intake+RampUp2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT)
                                         .alongWith(CatzSuperstructure.Instance.cmdHoardStandby()));
        traj2.atTime("Hoard2").onTrue(CatzSuperstructure.Instance.cmdHoardShoot());
        traj6.atTime("HoardStop6").onTrue(CatzSuperstructure.Instance.cmdShooterStop());
        traj7.atTime("IntakeStop+RampUp9").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT)
                                         .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        traj8.atTime("Score8").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));
        prepRoutine(
            traj1,
            Commands.runOnce(() -> CommandScheduler.getInstance().schedule(CatzSuperstructure.Instance.deployIntake().alongWith(CatzSuperstructure.Instance.trackStaticHub()))),
            Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            followTrajectoryWithAccuracy(traj5),
            followTrajectoryWithAccuracy(traj6),
            followTrajectoryWithAccuracy(traj7),
            followTrajectoryWithAccuracy(traj8),
            followTrajectoryWithAccuracy(traj9),
            followTrajectoryWithAccuracy(traj10),
            Commands.print("Climb"),
            Commands.print("done")
        );
    }
}
