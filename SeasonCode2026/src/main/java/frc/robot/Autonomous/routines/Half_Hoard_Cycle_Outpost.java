package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

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
        AutoTrajectory traj13 = getTrajectory("Half_Hoard_Cycle_Outpost",12);
        AutoTrajectory traj14 = getTrajectory("Half_Hoard_Cycle_Outpost",13);


        traj2.atTime("Intake+RampUp2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT)
                                         .alongWith(CatzSuperstructure.Instance.cmdHoardStandby()));
        traj3.atTime("Hoard3").onTrue(CatzSuperstructure.Instance.cmdHoardShoot());
        traj6.atTime("HoardStop6").onTrue(CatzSuperstructure.Instance.cmdShooterStop());
        traj8.atTime("RampUp+IntakeStop8").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                         .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        traj10.atTime("Score10").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));
        traj11.atTime("Intake11").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj13.atTime("RampUp+IntakeStop13").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                         .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        prepRoutine(
            traj1,
            Commands.runOnce(() -> CommandScheduler.getInstance().schedule(CatzSuperstructure.Instance.deployIntake().alongWith(CatzSuperstructure.Instance.trackStaticHub()))),
            Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
            followTrajectory(traj1),
            followTrajectory(traj2),
            followTrajectory(traj3),
            followTrajectory(traj4),
            followTrajectory(traj5),
            followTrajectory(traj6),
            followTrajectory(traj7),
            followTrajectory(traj8),
            followTrajectory(traj9),
            followTrajectory(traj10),
            followTrajectory(traj11),
            followTrajectory(traj12),
            followTrajectory(traj13),
            followTrajectory(traj14),
            shootAllBalls(AutonConstants.PRELOAD_SHOOTING_WAIT),
            Commands.print("done")
        );
    }
}
