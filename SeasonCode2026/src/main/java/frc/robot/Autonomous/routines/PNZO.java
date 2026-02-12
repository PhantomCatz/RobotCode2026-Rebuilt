package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

public class PNZO extends AutoRoutineBase{
    public PNZO(){
        super("PNZO");

        AutoTrajectory traj1 = getTrajectory("PNZO",0);
        AutoTrajectory traj2 = getTrajectory("PNZO",1);
        AutoTrajectory traj3 = getTrajectory("PNZO",2);

        traj1.atTime("Score1").onTrue(CatzSuperstructure.Instance.prepareForShooting());
        traj1.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.MAX_SPEED));
        traj3.atTime("RampUp+StopIntake3").onTrue(CatzSuperstructure.Instance.interpolateFlywheelSpeed()
                                                    .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)));
        traj3.atTime("Score3").onTrue(CatzSuperstructure.Instance.prepareForShooting());
        traj3.atTime("RampUp+Intake").onTrue(CatzSuperstructure.Instance.interpolateFlywheelSpeed()
                                               .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.MAX_SPEED)));

        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            CatzSuperstructure.Instance.prepareForShooting(),
            Commands.print("done")

        );
    }
}
