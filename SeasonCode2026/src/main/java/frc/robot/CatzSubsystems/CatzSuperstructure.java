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
import frc.robot.CatzSubsystems.CatzShooter.AimCalculations.HoardTargetType;
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
    private HoardTargetType currentHoardType = HoardTargetType.RELATIVE_CLOSE;
    private boolean isScoring = false;

    private boolean initialShootReady = false;
    private RegressionMode activeRegressionMode = RegressionMode.HUB;

    private CatzSuperstructure() {}

    private Translation2d getBaseTargetLocation(boolean isHub) {
        if (isHub) {
            return FieldConstants.getHubLocation();
        } else {
            return AimCalculations.getCornerHoardingTarget(currentHoardType);
        }
    }

    private RegressionMode calculateDynamicMode(boolean isHub) {
        if (isHub) return RegressionMode.HUB;
        return AimCalculations.getHoardRegressionMode(getBaseTargetLocation(false));
    }

    public void updateAndApplyShooterState(boolean isHub, boolean isShooting) {
        RegressionMode currentMode = calculateDynamicMode(isHub);
        Translation2d baseTarget = getBaseTargetLocation(isHub);
        Translation2d targetLoc = AimCalculations.calculateAndGetPredictedTargetLocation(baseTarget, currentMode);
        Distance dist = Units.Meters.of(targetLoc.getDistance(CatzTurret.Instance.getFieldToTurret()));

        if (activeRegressionMode != currentMode) {
            initialShootReady = false;
            activeRegressionMode = currentMode;
        }

        CatzTurret.Instance.applySetpoint(AimCalculations.calculateTurretTrackingSetpoint(targetLoc));
        CatzFlywheels.Instance.applySetpoint(ShooterRegression.getShooterSetpoint(dist, currentMode));

        if (isShooting) {
            CatzHood.Instance.applySetpoint(ShooterRegression.getHoodSetpoint(dist, currentMode));

            if (!initialShootReady && AimCalculations.readyToShoot()) {
                initialShootReady = true;
            }

            if (initialShootReady && CatzTurret.Instance.nearPositionSetpoint()) { //check for turret because turret can wrap. 
                CatzSpindexer.Instance.applySetpoint(SpindexerConstants.ON);
                CatzYdexer.Instance.applySetpoint(Setpoint.withVoltageSetpoint(YdexerConstants.SPEED.get()));
            } else {
                CatzSpindexer.Instance.applySetpoint(SpindexerConstants.OFF);
                CatzYdexer.Instance.applySetpoint(YdexerConstants.OFF);
            }
        } else {
            CatzHood.Instance.applySetpoint(HoodConstants.HOOD_STOW_SETPOINT);
            CatzSpindexer.Instance.applySetpoint(SpindexerConstants.OFF);
            CatzYdexer.Instance.applySetpoint(YdexerConstants.OFF);
            initialShootReady = false; 
        }
    }

    // --------------------------------------------------------------------------
    // Public Command States
    // --------------------------------------------------------------------------

    public Command cmdShooterStop() {
        return Commands.parallel(
            CatzFlywheels.Instance.setpointCommand(FlywheelConstants.OFF_SETPOINT),
            CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOW_SETPOINT),
            CatzSpindexer.Instance.setpointCommand(SpindexerConstants.OFF),
            CatzYdexer.Instance.setpointCommand(YdexerConstants.OFF),
            Commands.runOnce(() -> initialShootReady = false),
            Commands.runOnce(() -> isScoring = false)
        );
    }

    public Command trackStaticHub() {
        return CatzTurret.Instance.followSetpointCommand(() -> AimCalculations.calculateHubTrackingSetpoint());
    }

    /* --- HOARDING --- */

    public Command cmdHoardShoot() {
        return Commands.run(() -> {
            updateAndApplyShooterState(false, true);
        }, CatzTurret.Instance, CatzFlywheels.Instance, CatzHood.Instance, CatzSpindexer.Instance, CatzYdexer.Instance);
    }

    public Command cmdHoardStandby() {
        return Commands.run(() -> {
            updateAndApplyShooterState(false, false);
        }, CatzTurret.Instance, CatzFlywheels.Instance, CatzHood.Instance, CatzSpindexer.Instance, CatzYdexer.Instance);
    }

    /* --- HUB SCORING --- */

    public Command cmdHubShoot() {
        return Commands.run(() -> {
            updateAndApplyShooterState(true, true);
        }, CatzTurret.Instance, CatzFlywheels.Instance, CatzHood.Instance, CatzSpindexer.Instance, CatzYdexer.Instance)
        .beforeStarting(() -> isScoring = true);
    }

    public Command cmdHubStandby() {
        return Commands.run(() -> {
            updateAndApplyShooterState(true, false);
        }, CatzTurret.Instance, CatzFlywheels.Instance, CatzHood.Instance, CatzSpindexer.Instance, CatzYdexer.Instance)
        .beforeStarting(() -> isScoring = false);
    }

    public Command toggleHoardLocation() {
        return Commands.runOnce(() -> {
            if (currentHoardType == HoardTargetType.RELATIVE_CLOSE) {
                currentHoardType = HoardTargetType.RELATIVE_FAR;
            } else {
                currentHoardType = HoardTargetType.RELATIVE_CLOSE;
            }
        });
    }

    public void setAbsoluteHoardingType(HoardTargetType type) {
        this.currentHoardType = type;
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

    public Command deployIntake(){
        return CatzIntakeDeploy.Instance.setpointCommand(IntakeDeployConstants.DEPLOY).alongWith(Commands.runOnce(()-> isIntakeDeployed = true));
    }

    public Command stowIntake(){
        return CatzIntakeDeploy.Instance.setpointCommand(IntakeDeployConstants.STOW).alongWith(Commands.runOnce(()-> isIntakeDeployed = false));
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
