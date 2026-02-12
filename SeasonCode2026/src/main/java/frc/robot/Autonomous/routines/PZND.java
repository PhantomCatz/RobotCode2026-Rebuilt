package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

public class PZND extends AutoRoutineBase{
    public PZND(){
        super("PZND");

        AutoTrajectory traj1 = getTrajectory("PZND",0);
        AutoTrajectory traj2 = getTrajectory("PZND",1);
        AutoTrajectory traj3 = getTrajectory("PZND",2);

        traj1.atTime("Score1").onTrue(CatzSuperstructure.Instance.prepareForShooting());
        traj1.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.MAX_SPEED));
        traj2.atTime("StopIntake+RampUp2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                                    .alongWith(CatzSuperstructure.Instance.interpolateFlywheelSpeed()));
        traj3.atTime("Score3").onTrue(CatzSuperstructure.Instance.prepareForShooting());
        traj3.atTime("Intake+RampUp4").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.MAX_SPEED)
                                                .alongWith(CatzSuperstructure.Instance.interpolateFlywheelSpeed()));

        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            Commands.print("Score4")
        );
    }
}
