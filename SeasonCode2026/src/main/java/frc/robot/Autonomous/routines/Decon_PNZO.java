package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Decon_PNZO extends AutoRoutineBase{
    public Decon_PNZO(){
        super("Decon_PNZO");

        AutoTrajectory traj1 = getTrajectory("Decon_PNZO",0);
        AutoTrajectory traj2 = getTrajectory("Decon_PNZO",1);
        // AutoTrajectory traj3 = getTrajectory("Decon_PNZO",2);
        // AutoTrajectory traj4 = getTrajectory("Decon_PNZO",3);
        AutoTrajectory traj3 = getTrajectory("Decon_PNZO",2);
        AutoTrajectory traj4 = getTrajectory("Decon_PNZO",3);

        // traj2.atTime("Intake2").onTrue();
        // traj5.atTime("RampUp+StopIntake5").onTrue(CatzSuperstructure.Instance.cmdHubStandby()
        //                                        .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT)));
        // traj6.atTime("RampUp+Intake6").onTrue(CatzSuperstructure.Instance.cmdHubStandby()
        //                                             .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)));
        // traj6.atTime("Score6").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));
        // traj6.atTime("Score7").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));

        prepRoutine(
            traj1,
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectory(traj1),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj2),
                    CatzSuperstructure.Instance.intakeOFF(),
                    followTrajectory(traj3)
                ),
                CatzSuperstructure.Instance.trackStaticHub()
            ),
            Commands.deadline(
                followTrajectory(traj4),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.print("done")

        );
    }
}
