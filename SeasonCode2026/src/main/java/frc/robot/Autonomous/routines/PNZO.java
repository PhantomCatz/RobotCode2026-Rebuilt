package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

public class PNZO extends AutoRoutineBase{
    public PNZO(){
        super("PNZO");

        AutoTrajectory traj1 = getTrajectory("PNZO",0);
        AutoTrajectory traj2 = getTrajectory("PNZO",1);
        AutoTrajectory traj3 = getTrajectory("PNZO",2);
        AutoTrajectory traj4 = getTrajectory("PNZO",3);
        AutoTrajectory traj5 = getTrajectory("PNZO",4);
        AutoTrajectory traj6 = getTrajectory("PNZO",5);

        traj2.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj5.atTime("RampUp+StopIntake5").onTrue(CatzSuperstructure.Instance.cmdHubStandby()
                                               .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT)));        
        traj6.atTime("RampUp+Intake6").onTrue(CatzSuperstructure.Instance.cmdHubStandby()
                                                    .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)));
        traj6.atTime("Score6").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));
        traj6.atTime("Score7").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));

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
            Commands.print("done")

        );
    }
}