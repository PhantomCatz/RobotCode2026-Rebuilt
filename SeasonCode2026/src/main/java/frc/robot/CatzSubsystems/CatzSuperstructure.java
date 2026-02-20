package frc.robot.CatzSubsystems;

import java.util.Set;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer.CatzSpindexer;
import frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer.SpindexerConstants;
import frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer.CatzYdexer;
import frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer.YdexerConstants;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeDeploy.CatzIntakeDeploy;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeDeploy.IntakeDeployConstants;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
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

    public boolean isClimbMode = false;

    private boolean isCloseCornerHoarding = true;
    private boolean isScoring = false;

    private CatzSuperstructure() {}

    private Translation2d getTargetLocation(RegressionMode mode) {
        if (mode == RegressionMode.HUB) {
            return AimCalculations.getPredictedHubLocation();
        } else {
            return AimCalculations.getCornerHoardingTarget(isCloseCornerHoarding);
        }
    }

    //TODO this is a bad way to do it find a better way lol
    private Translation2d getTargetLocationFlywheel(RegressionMode mode) {
        if (mode == RegressionMode.HUB) {
            return AimCalculations.calculateAndGetPredictedHubLocation();
        } else {
            return AimCalculations.getCornerHoardingTarget(isCloseCornerHoarding);
        }
    }

    private RegressionMode getSpecificMode(RegressionMode mode) {
        if (mode != RegressionMode.HUB) {
            return isCloseCornerHoarding ? RegressionMode.CLOSE_HOARD : RegressionMode.FAR_HOARD;
        }
        return mode;
    }

    /**
     * Tracks the target using the Turret.
     */
    public Command trackTarget(RegressionMode mode) {
        return Commands.run(() -> {
            Translation2d targetLoc = getTargetLocation(mode);
            CatzTurret.Instance.applySetpoint(AimCalculations.calculateTurretTrackingSetpoint(targetLoc));
        }, CatzTurret.Instance);
    }

    public Command trackStaticHub(){
        return CatzTurret.Instance.followSetpointCommand(() -> AimCalculations.calculateHubTrackingSetpoint());
    }

    /**
     * Ramps up Flywheels based on distance to target.
     */
    public Command rampUpFlywheels(RegressionMode mode) {
        return Commands.run(() -> {
            Translation2d targetLoc = getTargetLocationFlywheel(mode);
            Translation2d turretPos = CatzTurret.Instance.getFieldToTurret();
            Distance dist = Units.Meters.of(targetLoc.getDistance(turretPos));

            RegressionMode specificMode = getSpecificMode(mode);

            CatzFlywheels.Instance.applySetpoint(ShooterRegression.getShooterSetpoint(dist, specificMode));
        }, CatzFlywheels.Instance);
    }

    /**
     * Updates Hood Angle based on the target mode.
     */
    private Command aimHood(RegressionMode mode) {
        return Commands.run(() -> {
            Translation2d targetLoc = getTargetLocation(mode);

            Distance dist = Units.Meters.of(targetLoc.getDistance(CatzTurret.Instance.getFieldToTurret()));

            RegressionMode specificMode = getSpecificMode(mode);

            CatzHood.Instance.applySetpoint(ShooterRegression.getHoodSetpoint(dist, specificMode));
        }, CatzHood.Instance);
    }

    /**
     * Feeds balls to shooter when ready.
     */
    private boolean initialShootReady = false;
    private Command runFeeder() {
        return Commands.run(() -> {
            if(!initialShootReady && AimCalculations.readyToShoot()) {
                initialShootReady = true;
            }

            if (initialShootReady) { //TODO At least check if turret angle is correct because it can wrap.
                CatzSpindexer.Instance.applySetpoint(SpindexerConstants.ON);
                CatzYdexer.Instance.applySetpoint(Setpoint.withVoltageSetpoint(YdexerConstants.SPEED.get()));
            }
        }, CatzSpindexer.Instance, CatzYdexer.Instance);
    }

    // --------------------------------------------------------------------------
    // Public Command States
    // --------------------------------------------------------------------------

    // Stops everything but the turret
    public Command cmdFullStop() {
        return Commands.parallel(
            CatzFlywheels.Instance.setpointCommand(FlywheelConstants.OFF_SETPOINT),
            CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOW_SETPOINT),
            CatzSpindexer.Instance.setpointCommand(SpindexerConstants.OFF),
            CatzYdexer.Instance.setpointCommand(YdexerConstants.OFF),
            Commands.runOnce(() -> initialShootReady = false),
            Commands.runOnce(() -> isScoring = false)
        );
    }

    /* --- HOARDING --- */

    public Command cmdHoardShoot() {
        return Commands.parallel(
            trackTarget(RegressionMode.CLOSE_HOARD),  //The arguments are placeholders to indicate hoarding.
            rampUpFlywheels(RegressionMode.CLOSE_HOARD),//Actual modes are decided within the method
            aimHood(RegressionMode.CLOSE_HOARD),
            runFeeder()
        );
    }

    public Command cmdHoardStandby() {
        return Commands.parallel(
            trackTarget(RegressionMode.CLOSE_HOARD),
            rampUpFlywheels(RegressionMode.CLOSE_HOARD),
            CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOW_SETPOINT),
            CatzSpindexer.Instance.setpointCommand(SpindexerConstants.OFF),
            CatzYdexer.Instance.setpointCommand(YdexerConstants.OFF),
            Commands.runOnce(() -> initialShootReady = false)
        );
    }

    /* --- HUB SCORING --- */

    public Command cmdHubShoot() {
        return Commands.parallel(
            rampUpFlywheels(RegressionMode.HUB),
            trackTarget(RegressionMode.HUB),
            aimHood(RegressionMode.HUB),
            runFeeder(),
            Commands.runOnce(() -> isScoring = true)
        );
    }

    public Command cmdHubStandby() {
        return Commands.parallel(
            rampUpFlywheels(RegressionMode.HUB),
            trackTarget(RegressionMode.HUB),
            CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOW_SETPOINT),
            CatzSpindexer.Instance.setpointCommand(SpindexerConstants.OFF),
            CatzYdexer.Instance.setpointCommand(YdexerConstants.OFF),
            Commands.runOnce(() -> isScoring = false),
            Commands.runOnce(() -> initialShootReady = false)
        );
    }

    public Command toggleHoardLocation() {
        return Commands.runOnce(() -> {
            isCloseCornerHoarding = !isCloseCornerHoarding;
        });
    }

    /* --- INTAKE --- */
    private boolean isIntakeDeployed = false;
    private boolean isIntakeOn = false;

    public Command toggleIntakeDeploy() {
        return Commands.runOnce(() -> {
            if(isIntakeDeployed){
                isIntakeDeployed = false;
                CatzIntakeDeploy.Instance.applySetpoint(IntakeDeployConstants.STOW);
            }else{
                isIntakeDeployed = true;
                CatzIntakeDeploy.Instance.applySetpoint(IntakeDeployConstants.DEPLOY);
            }
        }, CatzIntakeDeploy.Instance);
    }

    public Command toggleIntakeRollers() {
        return Commands.runOnce(() -> {
            if(isIntakeOn){
                isIntakeOn = false;
                CatzIntakeRoller.Instance.applySetpoint(IntakeRollerConstants.OFF_SETPOINT);
            }else{
                isIntakeOn = true;
                CatzIntakeRoller.Instance.applySetpoint(Setpoint.withDutyCycleSetpoint(IntakeRollerConstants.TUNABLE_PERCENT.get()));
// CatzIntakeRoller.Instance.applySetpoint(IntakeRollerConstants.S_SETPOINT);
            }
        }, CatzIntakeRoller.Instance);
    }

    public boolean getIsScoring(){
        return isScoring;
    }

    /* FUNCTIONAL COMMANDS */
    private boolean isSpindexerSpinning = false;
    public Command toggleSpindexer() {
        return Commands.runOnce(() -> {
            if(isSpindexerSpinning){
                isSpindexerSpinning = false;
                CatzSpindexer.Instance.applySetpoint(SpindexerConstants.OFF);
            }else{
                isSpindexerSpinning = true;
                CatzSpindexer.Instance.applySetpoint(SpindexerConstants.ON);
            }
        }, CatzSpindexer.Instance);
    }

    private boolean isYdexerSpinning = false;
    public Command toggleYdexer() {
        return Commands.runOnce(() -> {
            if(isYdexerSpinning){
                isYdexerSpinning = false;
                CatzYdexer.Instance.applySetpoint(YdexerConstants.OFF);
            }else{
                isYdexerSpinning = true;
                CatzYdexer.Instance.applySetpoint(YdexerConstants.ON);
            }
        }, CatzYdexer.Instance);
    }

    private boolean isFlywheelSpinning = false;
    public Command toggleFlywheel() {
        return Commands.runOnce(() -> {
            if(isFlywheelSpinning){
                isFlywheelSpinning = false;
                CatzFlywheels.Instance.applySetpoint(FlywheelConstants.OFF_SETPOINT);
            }else{
                isFlywheelSpinning = true;
                CatzFlywheels.Instance.applySetpoint(FlywheelConstants.TEST_SETPOINT);
            }
        }, CatzFlywheels.Instance);
    }

    private boolean isTurretAtZero = true;
    public Command toggleTurret() {
        return Commands.runOnce(() -> {
            if(isTurretAtZero){
                isTurretAtZero = false;
                CatzTurret.Instance.applySetpoint(Setpoint.withMotionMagicSetpoint(Units.Degrees.of(90)));
            }else{
                isTurretAtZero = true;
                CatzTurret.Instance.applySetpoint(Setpoint.withMotionMagicSetpoint(Units.Degrees.of(0)));
            }
        }, CatzTurret.Instance);
    }

    private boolean isHoodAtHome = true;
    public Command toggleHood() {
        return Commands.runOnce(() -> {
            if(isHoodAtHome){
                isHoodAtHome = false;
                CatzHood.Instance.applySetpoint(HoodConstants.HOOD_STOW_SETPOINT);
            }else{
                isHoodAtHome = true;
                CatzHood.Instance.applySetpoint(HoodConstants.HOOD_TEST_SETPOINT);
            }
        }, CatzHood.Instance);
    }

    /* --- REVERSIONARY MODES --- */

    public Command setHoodHomePosition() {
        return Commands.parallel(
            CatzHood.Instance.setpointCommand(() -> HoodConstants.HOOD_HOME_SETPOINT),
            Commands.waitSeconds(1.0).andThen(() -> CatzHood.Instance.setCurrentPosition(Units.Degree.of(0)))
            .andThen(() -> CatzHood.Instance.setpointCommand(() -> HoodConstants.HOOD_STOP))
        );
    }

    /* --- COMMANDS FOR TESTING --- */

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

    // public Command turretManualCommand() {
    //     return CatzTurret.Instance.followSetpointCommand(() -> {

    //     });
    // }

    public Command turret30Deg() {
        return CatzTurret.Instance.setpointCommand(Setpoint.withMotionMagicSetpoint(Units.Degrees.of(30.0)));
    }
    public Command turretMinus30Deg() {
        return CatzTurret.Instance.setpointCommand(Setpoint.withMotionMagicSetpoint(Units.Degrees.of(-30.0)));
    }

    public Command turretTrackHubCommand() {
        return CatzTurret.Instance.followSetpointCommand(() -> AimCalculations.calculateHubTrackingSetpoint());
    }

    public Command alignToBackUpClimb(boolean isRight) {
        return new PIDDriveCmd(FieldConstants.getClimbBackAwayPosition(isRight), true).onlyIf(()->isClimbMode);
    }

    public Command alignToCloseClimb(boolean isRight) {
        return new PIDDriveCmd(FieldConstants.getClimbClosePosition(isRight), true).onlyIf(()->isClimbMode);
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
