package frc.robot.CatzSubsystems;

import java.util.Set;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer.CatzSpindexer;
import frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer.SpindexerConstants;
import frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer.CatzYdexer;
import frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer.YdexerConstants;
import frc.robot.CatzSubsystems.CatzShooter.AimCalculations;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels.CatzFlywheels;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels.FlywheelConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzHood.CatzHood;
import frc.robot.CatzSubsystems.CatzShooter.CatzHood.HoodConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression.RegressionMode;
import frc.robot.Commands.DriveAndRobotOrientationCmds.PIDDriveCmd;
import frc.robot.Utilities.Setpoint;

public class CatzSuperstructure {
    public static final CatzSuperstructure Instance = new CatzSuperstructure();

    private boolean isCloseCornerHoarding = true;

    private CatzSuperstructure() {}

    public Command trackTargetAndRampUp(RegressionMode mode) {
        return Commands.run(() -> {
            Translation2d targetLoc;

            if (mode == RegressionMode.HUB) {
                targetLoc = AimCalculations.getPredictedHubLocation();
            } else {
                //The only other states are hoarding states
                targetLoc = AimCalculations.getCornerHoardingTarget(isCloseCornerHoarding);
            }

            Translation2d turretPos = CatzTurret.Instance.getFieldToTurret();
            Distance dist = Units.Meters.of(targetLoc.getDistance(turretPos));

            CatzTurret.Instance.applySetpoint(AimCalculations.calculateTurretTrackingSetpoint(targetLoc));
            
            RegressionMode specificMode = mode;
            if (mode != RegressionMode.HUB) {
                specificMode = isCloseCornerHoarding ? RegressionMode.CLOSE_HOARD : RegressionMode.OPP_HOARD;
            }

            CatzFlywheels.Instance.applySetpoint(ShooterRegression.getShooterSetpoint(dist, specificMode));

        }, CatzFlywheels.Instance, CatzTurret.Instance);
    }

    /**
     * Updates Hood Angle based on the target mode.
     */
    private Command aimHood(RegressionMode mode) {
        return Commands.run(() -> {
            Translation2d targetLoc = (mode == RegressionMode.HUB) ?
                AimCalculations.getPredictedHubLocation() :
                AimCalculations.getCornerHoardingTarget(isCloseCornerHoarding);

            Distance dist = Units.Meters.of(targetLoc.getDistance(CatzTurret.Instance.getFieldToTurret()));

            RegressionMode specificMode = mode;
            if (mode != RegressionMode.HUB) {
                specificMode = isCloseCornerHoarding ? RegressionMode.CLOSE_HOARD : RegressionMode.OPP_HOARD;
            }

            CatzHood.Instance.applySetpoint(ShooterRegression.getHoodSetpoint(dist, specificMode));
        }, CatzHood.Instance);
    }

    /**
     * Feeds balls to shooter when ready.
     */
    private Command runFeeder() {
        return Commands.run(() -> {
            if (AimCalculations.readyToShoot()) {
                CatzSpindexer.Instance.applySetpoint(SpindexerConstants.ON);
                CatzYdexer.Instance.applySetpoint(Setpoint.withVoltageSetpoint(YdexerConstants.SPEED.get()));
            } else {
                CatzSpindexer.Instance.applySetpoint(SpindexerConstants.OFF);
                CatzYdexer.Instance.applySetpoint(YdexerConstants.OFF);
            }
        }, CatzSpindexer.Instance, CatzYdexer.Instance);
    }

    // --------------------------------------------------------------------------
    // Public Command States
    // --------------------------------------------------------------------------

    //Stops everything but the turret
    public Command cmdFullStop() {
        return Commands.parallel(
            CatzFlywheels.Instance.setpointCommand(FlywheelConstants.OFF_SETPOINT),
            CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOW_SETPOINT),
            CatzSpindexer.Instance.setpointCommand(SpindexerConstants.OFF),
            CatzYdexer.Instance.setpointCommand(YdexerConstants.OFF)
        );
    }

    /* --- HOARDING --- */

    public Command cmdHoardShoot() {
        return Commands.parallel(
            trackTargetAndRampUp(RegressionMode.CLOSE_HOARD), // Mode argument is placeholder, logic handles Close/Opp
            aimHood(RegressionMode.CLOSE_HOARD),
            runFeeder()
        );
    }

    public Command cmdHoardStandby() {
        return Commands.parallel(
            trackTargetAndRampUp(RegressionMode.CLOSE_HOARD),
            CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOW_SETPOINT),
            CatzSpindexer.Instance.setpointCommand(SpindexerConstants.OFF),
            CatzYdexer.Instance.setpointCommand(YdexerConstants.OFF)
        );
    }

    /* --- HUB SCORING --- */

    public Command cmdHubShoot() {
        return Commands.parallel(
            trackTargetAndRampUp(RegressionMode.HUB),
            aimHood(RegressionMode.HUB),
            runFeeder()
        );
    }

    public Command cmdHubStandby() {
        return Commands.parallel(
            trackTargetAndRampUp(RegressionMode.HUB),
            CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOW_SETPOINT),
            CatzSpindexer.Instance.setpointCommand(SpindexerConstants.OFF),
            CatzYdexer.Instance.setpointCommand(YdexerConstants.OFF)
        );
    }

    public Command toggleHoardLocation() {
        return Commands.runOnce(() -> {
            isCloseCornerHoarding = !isCloseCornerHoarding;
        });
    }

    /* --- COMMANDS FOR TESTING */

    public Command applyHoodTuningSetpoint() {
        return Commands.defer(() -> {
            Angle angle = Units.Degrees.of(HoodConstants.adjustableHoodAngle.get());

            return CatzHood.Instance.followSetpointCommand(() -> Setpoint.withMotionMagicSetpoint(angle));
        }, Set.of(CatzHood.Instance));
    }

    public Command applyFlywheelTuningSetpoint() {
        return Commands.defer(() -> {

            return CatzFlywheels.Instance.setpointCommand(
                    Setpoint.withVelocitySetpointVoltage((FlywheelConstants.SHOOTING_RPS_TUNABLE.get())));
        }, Set.of(CatzFlywheels.Instance));
    }

    public Command turretTrackHubCommand() {
        return CatzTurret.Instance.followSetpointCommand(() -> AimCalculations.calculateHubTrackingSetpoint());
    }

    public Command alignToBackUpClimb(boolean isRight) {
        return new PIDDriveCmd(FieldConstants.getClimbBackAwayPosition(isRight), true);
    }

    public Command alignToCloseClimb(boolean isRight) {
        return new PIDDriveCmd(FieldConstants.getClimbClosePosition(isRight), true);
    }

    public Command alignToClimb(boolean isRight) {
        return Commands.deadline(
            Commands.sequence(
                alignToBackUpClimb(isRight),
                alignToCloseClimb(isRight)
            ),
            CatzTurret.Instance.followSetpointCommand(() -> AimCalculations.calculateTurretTrackingSetpoint(FieldConstants.getClimbTurretTrackingLocation()))
        );
    }
}
