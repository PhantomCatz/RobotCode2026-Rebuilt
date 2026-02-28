package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

public class R3_IAS extends AutoRoutineBase{
    public R3_IAS(){
        super("R3_IAS");

/*
 *
 *
 *
 * Out of service - we gave up on depot stuff cuz drivetrain too low
 *
 *
 *
 */

        AutoTrajectory traj1 = getTrajectory("R3_IAS",0);
        AutoTrajectory traj2 = getTrajectory("R3_IAS",1);
        AutoTrajectory traj3 = getTrajectory("R3_IAS",2);
        AutoTrajectory traj4 = getTrajectory("R3_IAS",3);

        traj1.atTime("Score1").onTrue(CatzSuperstructure.Instance.cmdHubShoot());
        traj1.atTime("Intake+RampUp2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT)
                                                .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        traj2.atTime("Score2").onTrue(CatzSuperstructure.Instance.cmdHubShoot());
        traj2.atTime("StopIntake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT));
        traj3.atTime("Intake+RampUp4").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT)
                                                .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        traj4.atTime("StopIntake+Score4").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                                   .alongWith(CatzSuperstructure.Instance.cmdHubShoot()));

        prepRoutine(
            traj1,
            Commands.runOnce(() -> CommandScheduler.getInstance().schedule(CatzSuperstructure.Instance.deployIntake().alongWith(CatzSuperstructure.Instance.trackStaticHub()))),
            Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            Commands.print("Climb5"),
            Commands.print("done")
        );
    }
}
