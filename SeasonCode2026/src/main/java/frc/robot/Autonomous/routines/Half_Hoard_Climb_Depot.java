package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

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
        AutoTrajectory traj10 = getTrajectory("Half_Hoard_Climb_Depot",9);
        AutoTrajectory traj11 = getTrajectory("Half_Hoard_Climb_Depot",10);

        traj2.atTime("RampUp+Intake2").onTrue(CatzSuperstructure.Instance.intakeON()
                                         .alongWith(CatzSuperstructure.Instance.cmdHoardStandby()));
        traj3.atTime("Hoard3").onTrue(CatzSuperstructure.Instance.cmdHoardShoot());
        traj6.atTime("HoardStop6").onTrue(CatzSuperstructure.Instance.cmdShooterStop());
        traj8.atTime("RampUp+IntakeStop8").onTrue(CatzSuperstructure.Instance.intakeOFF()
                                                    .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        traj10.atTime("Score10").onTrue(CatzSuperstructure.Instance.cmdHubShoot());


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
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            followTrajectoryWithAccuracy(traj11),
            Commands.print("Climb"), //TODO
            Commands.print("done")
        );
    }
}
