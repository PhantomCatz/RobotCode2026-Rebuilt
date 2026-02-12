package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

public class Outpostclimb extends AutoRoutineBase{
    public Outpostclimb(){
        super("Outpostclimb");
//Warning if this crashes its jadens fault
        AutoTrajectory traj1 = getTrajectory("Outpostclimb",0);
        AutoTrajectory traj2 = getTrajectory("Outpostclimb",1);
        AutoTrajectory traj3 = getTrajectory("Outpostclimb",2);
        AutoTrajectory traj4 = getTrajectory("Outpostclimb",3);

        traj1.atTime("Score1").onTrue(CatzSuperstructure.Instance.prepareForShooting()); //Commands.print("Score1"));
        traj1.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.MAX_SPEED)); //Commands.print("Intake2"));
        traj2.atTime("RampUp+IntakeStop2").onTrue(CatzSuperstructure.Instance.interpolateFlywheelSpeed()
                                                    .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT))); //Commands.print("RampUp+IntakeStop2"));
        traj3.atTime("Score3").onTrue(CatzSuperstructure.Instance.prepareForShooting()); //Commands.print("Score3"));

        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            Commands.print("Climb5"), //TODO
            Commands.print("done")
        );
    }
}
