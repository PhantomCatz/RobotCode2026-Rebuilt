package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzShooter.AimCalculations;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;

public class R2IAS extends AutoRoutineBase{
    public R2IAS(){
        super("R2IAS");

        AutoTrajectory traj1 = getTrajectory("R2IASOut");
        AutoTrajectory traj2 = getTrajectory("R2IASIn");

        traj1.atTime(AutonConstants.TURRET_TRACK).onTrue(CatzTurret.Instance.followSetpointCommand(() -> AimCalculations.calculateHubTrackingSetpoint()));

        traj1.atTime("TurretAim").onTrue(CatzTurret.Instance.followSetpointCommand(() -> AimCalculations.calculateHubTrackingSetpoint()));

        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1),
            Commands.waitSeconds(4.0),
            CatzSuperstructure.Instance.interpolateFlywheelSpeed().asProxy(),
            followTrajectoryWithAccuracy(traj2),
            CatzSuperstructure.Instance.shootIfReady()
        );
    }
}
