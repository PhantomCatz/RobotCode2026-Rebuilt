package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class PNZO extends AutoRoutineBase{
    public PNZO(){
        super("PNZO");

        AutoTrajectory traj1 = getTrajectory("PNZO",0);
        AutoTrajectory traj2 = getTrajectory("PNZO",1);
        AutoTrajectory traj3 = getTrajectory("PNZO",2);

        // traj2.atTime("Intake2").onTrue();
        // traj5.atTime("RampUp+StopIntake5").onTrue(CatzSuperstructure.Instance.cmdHubStandby()
        //                                        .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT)));
        // traj6.atTime("RampUp+Intake6").onTrue(CatzSuperstructure.Instance.cmdHubStandby()
        //                                             .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)));
        // traj6.atTime("Score6").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));
        // traj6.atTime("Score7").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));

        prepRoutine(
            traj1,
            CatzSuperstructure.Instance.deployIntake(),
            shootAllBallsNoJiggle(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj1),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj2),
                    CatzSuperstructure.Instance.intakeOFF()
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectoryWithAccuracy(traj3),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),

            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT + AutonConstants.PRELOAD_SHOOTING_WAIT + AutonConstants.OUTPOST_SCORING_WAIT),
            Commands.print("done")

        );
    }
}
