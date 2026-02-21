package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

public class Outpost_Climb extends AutoRoutineBase{
    public Outpost_Climb(){
        super("Outpost_Climb");
//Warning if this crashes its jadens fault
        AutoTrajectory traj1 = getTrajectory("Outpost_Climb",0);
        AutoTrajectory traj2 = getTrajectory("Outpost_Climb",1);
        AutoTrajectory traj3 = getTrajectory("Outpost_Climb",2);
        AutoTrajectory traj4 = getTrajectory("Outpost_Climb",3);

        traj1.atTime("Intake1").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj2.atTime("IntakeStop+RampUp2").onTrue(CatzSuperstructure.Instance.cmdHubStandby()
                                                    .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)));
        traj3.atTime("Score3").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));

        prepRoutine(
            traj1,
            CatzSuperstructure.Instance.deployIntake(),
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            Commands.print("Climb"), //TODO
            Commands.print("done")
        );
    }
}
