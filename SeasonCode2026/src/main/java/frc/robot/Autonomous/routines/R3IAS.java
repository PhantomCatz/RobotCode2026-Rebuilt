package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

public class R3IAS extends AutoRoutineBase{
    public R3IAS(){
        super("R3IAS");

        AutoTrajectory traj1 = getTrajectory("R3IAS",0);
        AutoTrajectory traj2 = getTrajectory("R3IAS",1);
        AutoTrajectory traj3 = getTrajectory("R3IAS",2);
        AutoTrajectory traj4 = getTrajectory("R3IAS",3);

        traj1.atTime("Score1").onTrue(CatzSuperstructure.Instance.prepareForShooting());
        traj1.atTime("Intake+RampUp2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.MAX_SPEED)
                                                .alongWith(CatzSuperstructure.Instance.interpolateFlywheelSpeed()));
        traj2.atTime("Score2").onTrue(CatzSuperstructure.Instance.prepareForShooting());
        traj2.atTime("StopIntake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT));
        traj3.atTime("Intake+RampUp4").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.MAX_SPEED)
                                                .alongWith(CatzSuperstructure.Instance.interpolateFlywheelSpeed()));
        traj4.atTime("StopIntake+Score4").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                                   .alongWith(CatzSuperstructure.Instance.prepareForShooting()));

        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            Commands.print("Climb5"),
            Commands.print("done")
        );
    }
}
