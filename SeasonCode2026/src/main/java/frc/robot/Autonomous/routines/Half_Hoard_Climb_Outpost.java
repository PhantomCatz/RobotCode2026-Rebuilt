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

        traj2.atTime("Intake+RampUp2").onTrue(CatzSuperstructure.Instance.intakeON()
                                         .alongWith(CatzSuperstructure.Instance.cmdHoardStandby()));
        traj4.atTime("Hoard4").onTrue(CatzSuperstructure.Instance.cmdHoardShoot());
        traj7.atTime("HoardStop7").onTrue(CatzSuperstructure.Instance.cmdShooterStop());
        traj8.atTime("RampUp+IntakeStop8").onTrue(CatzSuperstructure.Instance.intakeOFF()
                                         .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        traj9.atTime("Score+StowIntake+TrackTower9").onTrue(
                                                   CatzSuperstructure.Instance.stowIntake()
                                                   .alongWith(CatzSuperstructure.Instance.trackTower()));

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    followTrajectoryWithAccuracy(traj1),
                    followTrajectoryWithAccuracy(traj2),
                    followTrajectoryWithAccuracy(traj3),
                    followTrajectoryWithAccuracy(traj4),
                    followTrajectoryWithAccuracy(traj5),
                    followTrajectoryWithAccuracy(traj6)
                ),
                CatzSuperstructure.Instance.deployIntake().alongWith(CatzSuperstructure.Instance.trackStaticHub())
            ),
            Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
            followTrajectoryWithAccuracy(traj7),
            followTrajectoryWithAccuracy(traj8),
            Commands.deadline(
                Commands.waitSeconds(4), 
                shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT)
                    .alongWith(CatzSuperstructure.Instance.trackTower()
                    .alongWith(CatzSuperstructure.Instance.stowIntake()))
            ),
            followTrajectoryWithAccuracy(traj9),
            followTrajectoryWithAccuracy(traj10),
            Commands.print("Climb"),
            Commands.print("done")
        );
    }
}
