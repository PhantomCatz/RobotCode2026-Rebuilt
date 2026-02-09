package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

public class DepotClimb extends AutoRoutineBase{
    public DepotClimb(){
        super("DepotClimb");

        AutoTrajectory traj1 = getTrajectory("DepotClimb",0);
        AutoTrajectory traj2 = getTrajectory("DepotClimb",1);
        AutoTrajectory traj3 = getTrajectory("DepotClimb",2);
        AutoTrajectory traj4 = getTrajectory("DepotClimb",3);
        AutoTrajectory traj5 = getTrajectory("DepotClimb",4);

        traj1.atTime("Score1").onTrue(CatzSuperstructure.Instance.prepareForShooting());
        traj1.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.MAX_SPEED)
                                         .alongWith(Commands.print("intake deploy!"))); //TODO Intake deploy code not here yet
        traj3.atTime("RampUp+StopIntake3").onTrue(CatzSuperstructure.Instance.interpolateFlywheelSpeed()
                                                    .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)));
        traj3.atTime("Score4").onTrue(CatzSuperstructure.Instance.prepareForShooting());

        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            followTrajectoryWithAccuracy(traj5),
            Commands.print("Climb") //TODO
        );
    }
}
