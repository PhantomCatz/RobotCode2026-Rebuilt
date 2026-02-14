package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

public class R1IAS extends AutoRoutineBase{
    public R1IAS(){
        super("R1IAS");

        AutoTrajectory traj1 = getTrajectory("R1IAS",0);
        AutoTrajectory traj2 = getTrajectory("R1IAS",1);
        AutoTrajectory traj3 = getTrajectory("R1IAS",2);
        AutoTrajectory traj4 = getTrajectory("R1IAS",3);
        AutoTrajectory traj5 = getTrajectory("R1IAS",4);
        AutoTrajectory traj6 = getTrajectory("R1IAS",5);

        traj1.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.MAX_SPEED));
        traj1.atTime("StopIntake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT));
        traj2.atTime("RampUp3").onTrue(CatzSuperstructure.Instance.interpolateFlywheelSpeed());
        traj2.atTime("Score3").onTrue(CatzSuperstructure.Instance.prepareForShooting());
        traj3.atTime("Intake4").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.MAX_SPEED));
        traj3.atTime("StopIntake+RampUp").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT));
        traj4.atTime("Score5").onTrue(CatzSuperstructure.Instance.prepareForShooting());

        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            followTrajectoryWithAccuracy(traj5),
            followTrajectoryWithAccuracy(traj6),
            Commands.print("Climb"),
            Commands.print("done")
        );
    }
}
