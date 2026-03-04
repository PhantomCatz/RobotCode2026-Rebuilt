package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Half_Hoard_Cycle_Depot extends AutoRoutineBase{
    public Half_Hoard_Cycle_Depot(){
        super("Half_Hoard_Cycle_Depot");

        AutoTrajectory traj1 = getTrajectory("Half_Hoard_Cycle_Depot",0);
        AutoTrajectory traj2 = getTrajectory("Half_Hoard_Cycle_Depot",1);
        AutoTrajectory traj3 = getTrajectory("Half_Hoard_Cycle_Depot",2);
        AutoTrajectory traj4 = getTrajectory("Half_Hoard_Cycle_Depot",3);
        AutoTrajectory traj5 = getTrajectory("Half_Hoard_Cycle_Depot",4);
        AutoTrajectory traj6 = getTrajectory("Half_Hoard_Cycle_Depot",5);
        AutoTrajectory traj7 = getTrajectory("Half_Hoard_Cycle_Depot",6);
        AutoTrajectory traj8 = getTrajectory("Half_Hoard_Cycle_Depot",7);
        AutoTrajectory traj9 = getTrajectory("Half_Hoard_Cycle_Depot",8);
        AutoTrajectory traj10 = getTrajectory("Half_Hoard_Cycle_Depot",9);
        AutoTrajectory traj11 = getTrajectory("Half_Hoard_Cycle_Depot",10);
        AutoTrajectory traj12 = getTrajectory("Half_Hoard_Cycle_Depot",11);
        AutoTrajectory traj13 = getTrajectory("Half_Hoard_Cycle_Depot",12);
        AutoTrajectory traj14 = getTrajectory("Half_Hoard_Cycle_Depot",12);

        //I'm sorry william i lowkey don't know what I'm doing (sob emoji)
        //Allgood bro it works i think

        // traj2.atTime("RampUp+Intake2");
        // traj4.atTime("Hoard4").onTrue();
        // traj6.atTime("HoardStop6").onTrue();
        // traj8.atTime("IntakeStop+RampUp9").onTrue(CatzSuperstructure.Instance.intakeOFF()
                                                    // .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));
        // traj10.atTime("Score10").onTrue(shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT));
        // traj11.atTime("Intake11").onTrue(CatzSuperstructure.Instance.intakeON());
        // traj13.atTime("RampUp+IntakeStop13").onTrue(CatzSuperstructure.Instance.intakeOFF()
                                                    // .alongWith(CatzSuperstructure.Instance.cmdHubStandby()));

        prepRoutine(
            traj1,
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.deployIntake(),
                    Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
                    followTrajectory(traj1),
                    CatzSuperstructure.Instance.intakeON(),
                    followTrajectory(traj2)
                ),
                CatzSuperstructure.Instance.cmdHoardStandby()
            ),
            followTrajectory(traj3),
            followTrajectory(traj4),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj5),
                    followTrajectory(traj6)
                ),
                CatzSuperstructure.Instance.cmdShooterStop()
            ),
            followTrajectory(traj7),
            followTrajectory(traj8),
            Commands.deadline(
                Commands.sequence(
                    CatzSuperstructure.Instance.intakeOFF(),
                    followTrajectory(traj9)
                ),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            followTrajectory(traj10),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            followTrajectory(traj11),
            CatzSuperstructure.Instance.intakeON(),
            followTrajectory(traj12),
            followTrajectory(traj13),
            CatzSuperstructure.Instance.intakeOFF(),
            Commands.deadline(
                Commands.sequence(
                    followTrajectory(traj14)
                ),
                CatzSuperstructure.Instance.cmdHubStandby()
            ),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            Commands.print("done")
        );
    }
}
