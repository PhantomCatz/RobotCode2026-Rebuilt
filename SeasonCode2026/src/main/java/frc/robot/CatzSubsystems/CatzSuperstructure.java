package frc.robot.CatzSubsystems;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzClimb.ClimbConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer.CatzSpindexer;
import frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer.SpindexerConstants;
import frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer.CatzYdexer;
import frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer.YdexerConstants;
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

    private CatzSuperstructure() {
    }

    private Translation2d getBaseTargetLocation(boolean isHub) {
        if (isHub) {
            return FieldConstants.getHubLocation();
        } else {
            return AimCalculations.getCornerHoardingTarget(currentHoardType);
        }
    }

    private RegressionMode calculateDynamicMode(boolean isHub) {
        if (isHub)
            return RegressionMode.HUB;
        return AimCalculations.getHoardRegressionMode(getBaseTargetLocation(false));
    }

    private double getRumbleStrength() {
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
        if (currentPose.getY() > FieldConstants.BOTTOM_TRENCH_MAX_Y && currentPose.getY() < FieldConstants.TOP_TRENCH_MIN_Y) {
            return 0.0;
        }
        double distFromTrench = Math.min(Math.abs(currentPose.getX() - FieldConstants.LEFT_TRENCH_X), Math.abs(currentPose.getX() - FieldConstants.RIGHT_TRENCH_X));
        if (distFromTrench > FieldConstants.MIN_RUMBLE_DIST) return 0.0;
        return 1.0 - distFromTrench / FieldConstants.MIN_RUMBLE_DIST;
    }

    private boolean isIntakeOn = false;


    public void updateAndApplyShooterState(boolean isHub, boolean isShooting) {
        RegressionMode currentMode = calculateDynamicMode(isHub);
        Translation2d baseTarget = getBaseTargetLocation(isHub);

        Pose2d predictedRobotPose = AimCalculations.getPredictedRobotPose();
        Translation2d predictedTurretPose = CatzTurret.Instance.getFieldToTurret(predictedRobotPose);

        Translation2d targetLoc = AimCalculations.calculateAndGetPredictedTargetLocation(baseTarget, currentMode,
                predictedRobotPose, predictedTurretPose);
        Distance dist = Units.Meters.of(targetLoc.getDistance(predictedTurretPose));

        if (activeRegressionMode != currentMode) {
            initialShootReady = false;
            activeRegressionMode = currentMode;
        }

        CatzFlywheels.Instance.applySetpoint(ShooterRegression.getShooterSetpoint(dist, currentMode));

        if (isShooting) {
            CatzTurret.Instance
                    .applySetpoint(AimCalculations.calculateTurretTrackingSetpoint(targetLoc, predictedTurretPose, dist.in(Units.Meters)));

            if (isHub) {
                CatzHood.Instance.applySetpoint(Setpoint.withMotionMagicSetpoint(
                        AimCalculations.calculateHoodBisectorAngle(dist.in(Units.Meters)) / (2 * Math.PI)));
            } else {
                CatzHood.Instance.applySetpoint(ShooterRegression.getHoodSetpoint(dist, currentMode));
            }

            if (!initialShootReady && AimCalculations.readyToShoot()) {
                initialShootReady = true;
            }

            if (initialShootReady && CatzTurret.Instance.nearPositionSetpoint()) { // check for turret because turret
                                                                                   // can wrap.
                CatzSpindexer.Instance.applySetpoint(SpindexerConstants.ON);
                CatzYdexer.Instance.applySetpoint(Setpoint.withVoltageSetpoint(YdexerConstants.SPEED.get()));
            } else {
                CatzSpindexer.Instance.applySetpoint(SpindexerConstants.OFF);
                CatzYdexer.Instance.applySetpoint(YdexerConstants.OFF);
            }
            RobotContainer.rumbleDrv(getRumbleStrength());
            if(isIntakeOn){
                RobotContainer.rumbleDrv(0.05);
            }
        } else {
            CatzHood.Instance.applySetpoint(HoodConstants.HOOD_STOW_SETPOINT);
            CatzSpindexer.Instance.applySetpoint(SpindexerConstants.OFF);
            CatzYdexer.Instance.applySetpoint(YdexerConstants.OFF);
            CatzTurret.Instance.applySetpoint(AimCalculations.calculateTurretTrackingSetpoint(baseTarget));// don't aim
                                                                                                           // at future
                                                                                                           // pose
            initialShootReady = false;
            RobotContainer.rumbleDrv(0.0);
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
                Commands.runOnce(() -> isScoring = false),
                Commands.runOnce(() -> RobotContainer.rumbleDrv(0.0)));
    }

    public Command trackStaticHub() {
        return CatzTurret.Instance.followSetpointCommand(() -> AimCalculations.calculateHubTrackingSetpoint());
    }

    public Command trackTower() {
        return CatzTurret.Instance.followSetpointCommand(
                () -> AimCalculations.calculateTurretTrackingSetpoint(FieldConstants.getClimbApriltagLocation()));
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

    boolean toggleShooter = false;

    public Command toggleCmdHubShoot() {
        return Commands.runOnce(() -> {
            if (toggleShooter) {
                System.out.println("cmdHubShoot Start..."); // Debug
                cmdHubShoot().schedule();
                System.out.println("cmdHubShoot End..."); // Debug
            }
            else {
                System.out.println("Canceling..."); // Debug
                cmdShooterStop().schedule();
                System.out.println("Canceled..."); // Debug
            }
            toggleShooter = !toggleShooter;
        });
    }

    public Command cmdHubShoot() {
        return Commands.run(() -> {
                System.out.println("Applying updateAndApplyShooterState..."); // Debug
                updateAndApplyShooterState(true, true);
                System.out.println("Shooting..."); // Debug

        }, CatzTurret.Instance, CatzFlywheels.Instance, CatzHood.Instance, CatzSpindexer.Instance, CatzYdexer.Instance)
                .beforeStarting(() -> isScoring = true).alongWith(Commands.print("Shooting...")); // alongWith... is Debug
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

    public Command reverseIndexers(){
        return CatzSpindexer.Instance.setpointCommand(SpindexerConstants.REVERSE).alongWith(CatzYdexer.Instance.setpointCommand(YdexerConstants.REVERSE));
    }

    /* --- INTAKE --- */
    public Angle intakeSetpoint = IntakeDeployConstants.STOW_POSITION;
    public boolean isIntakeDeployed = false;

    // public Command toggleIntakeDeploy() {
    // return Commands.runOnce(() -> {
    // if(isIntakeDeployed){
    // isIntakeDeployed = false;
    // CatzIntakeDeploy.Instance.applySetpoint(IntakeDeployConstants.STOW);
    // }else{
    // isIntakeDeployed = true;
    // CatzIntakeDeploy.Instance.applySetpoint(IntakeDeployConstants.DEPLOY);
    // }
    // }, CatzIntakeDeploy.Instance);
    // }
    public Command toggleIntakeDeploy() {
        return Commands.runOnce(() -> {
            if (isIntakeDeployed) {
                intakeSetpoint = IntakeDeployConstants.STOW_POSITION;
            } else {
                intakeSetpoint = IntakeDeployConstants.DEPLOY_POSITION;
            }
            isIntakeDeployed = !isIntakeDeployed;
        });
    }

    public Command upIntake() {
        return Commands.runOnce(() -> intakeSetpoint = IntakeDeployConstants.UP_POSITION);
    }

    public Command deployIntake() {
        return Commands.runOnce(() -> {
            intakeSetpoint = IntakeDeployConstants.DEPLOY_POSITION;
            isIntakeDeployed = true;
        });
    }

    public Command stowIntake() {
        return Commands.runOnce(() -> {
            intakeSetpoint = IntakeDeployConstants.STOW_POSITION;
            isIntakeDeployed = false;
        });
    }

    public Command jiggleIntakeCommand() {
        Command jiggleCmd = Commands.run(() -> {
            double time = Timer.getFPGATimestamp();
            double angleRot = Math.sin(time * IntakeDeployConstants.JIGGLE_FREQUENCY_LOG.get() * (2 * Math.PI)) > 0
                    ? IntakeDeployConstants.JIGGLE_POSITION_LOG.get()
                    : IntakeDeployConstants.DEPLOY_POSITION.in(Units.Rotations);
            CatzIntakeRoller.Instance.applySetpoint(IntakeRollerConstants.JIGGLE_SETPOINT);
            intakeSetpoint = Units.Rotations.of(angleRot);

        });
        // TODO abuse of requirements. uses catz intake rollers to stop this command but
        // shouldn't do this
        jiggleCmd.addRequirements(CatzIntakeRoller.Instance);
        return jiggleCmd;
    }

    public Command toggleIntakeRollers() {
        return Commands.runOnce(() -> {
            if (isIntakeOn) {
                isIntakeOn = false;
                CatzIntakeRoller.Instance.applySetpoint(IntakeRollerConstants.OFF_SETPOINT);
                RobotContainer.rumbleDrv(0.0);
            } else {
                isIntakeOn = true;
                CatzIntakeRoller.Instance.applySetpoint(Setpoint.withVoltageSetpoint(IntakeRollerConstants.ON_SETPOINT_LOG.get()));
                RobotContainer.rumbleDrv(0.05);
                // CatzIntakeRoller.Instance.applySetpoint(IntakeRollerConstants.S_SETPOINT);
            }
        }, CatzIntakeRoller.Instance);
    }

    public Command intakeON() {
        return CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT)
                .alongWith(Commands.runOnce(() -> isIntakeOn = true));
    }

    public Command intakeOFF() {
        return CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                .alongWith(Commands.runOnce(() -> isIntakeOn = false));
    }

    public boolean getIsScoring() {
        return isScoring;
    }

    /* FUNCTIONAL COMMANDS */
    private boolean isSpindexerSpinning = false;

    public Command toggleSpindexer() {
        return Commands.runOnce(() -> {
            if (isSpindexerSpinning) {
                isSpindexerSpinning = false;
                CatzSpindexer.Instance.applySetpoint(SpindexerConstants.OFF);
            } else {
                isSpindexerSpinning = true;
                CatzSpindexer.Instance.applySetpoint(SpindexerConstants.ON);
            }
        }, CatzSpindexer.Instance);
    }

    private boolean isYdexerSpinning = false;

    public Command toggleYdexer() {
        return Commands.runOnce(() -> {
            if (isYdexerSpinning) {
                isYdexerSpinning = false;
                CatzYdexer.Instance.applySetpoint(YdexerConstants.OFF);
            } else {
                isYdexerSpinning = true;
                CatzYdexer.Instance.applySetpoint(YdexerConstants.ON);
            }
        }, CatzYdexer.Instance);
    }

    private boolean isFlywheelSpinning = false;

    public Command toggleFlywheel() {
        return Commands.runOnce(() -> {
            if (isFlywheelSpinning) {
                isFlywheelSpinning = false;
                CatzFlywheels.Instance.applySetpoint(FlywheelConstants.OFF_SETPOINT);
            } else {
                isFlywheelSpinning = true;
                CatzFlywheels.Instance.applySetpoint(FlywheelConstants.TEST_SETPOINT);
            }
        }, CatzFlywheels.Instance);
    }

    private boolean isTurretAtZero = true;

    public Command toggleTurret() {
        return Commands.runOnce(() -> {
            if (isTurretAtZero) {
                isTurretAtZero = false;
                CatzTurret.Instance.applySetpoint(Setpoint.withMotionMagicSetpoint(Units.Degrees.of(90)));
            } else {
                isTurretAtZero = true;
                CatzTurret.Instance.applySetpoint(Setpoint.withMotionMagicSetpoint(Units.Degrees.of(0)));
            }
        }, CatzTurret.Instance);
    }

    private boolean isHoodAtHome = true;

    public Command toggleHood() {
        return Commands.runOnce(() -> {
            if (isHoodAtHome) {
                isHoodAtHome = false;
                CatzHood.Instance.applySetpoint(HoodConstants.HOOD_TEST_SETPOINT);
            } else {
                isHoodAtHome = true;
                CatzHood.Instance.applySetpoint(HoodConstants.HOOD_STOW_SETPOINT);
            }
        }, CatzHood.Instance);
    }

    /* --- REVERSIONARY MODES --- */

    public Command setHoodHomePosition() {
        return Commands.parallel(
                CatzHood.Instance.setpointCommand(() -> HoodConstants.HOOD_HOME_SETPOINT),
                Commands.waitSeconds(1.0).andThen(() -> CatzHood.Instance.setCurrentPosition(Units.Degree.of(0)))
                        .andThen(() -> CatzHood.Instance.setpointCommand(() -> HoodConstants.HOOD_STOP)));
    }

    /* --- COMMANDS FOR TESTING --- */

    public Command applyHoodTuningSetpoint() {
        return Commands.defer(() -> {
            Angle angle = Units.Degrees.of(HoodConstants.adjustableHoodAngle.get());

            return CatzHood.Instance.followSetpointCommand(() -> Setpoint.withMotionMagicSetpoint(angle));
        }, Set.of(CatzHood.Instance));
    }

    public Command applyHoodInterpolatedSetpoint() {
        return CatzHood.Instance.followSetpointCommand(() -> {
            Distance dist = Units.Meters
                    .of(CatzTurret.Instance.getFieldToTurret().getDistance(FieldConstants.getHubLocation()));
            return ShooterRegression.getHoodSetpoint(dist, RegressionMode.HUB);
        });
    }

    public Command applyHoodBisectorSetpoint() {
        return CatzHood.Instance.followSetpointCommand(() -> {
            Distance dist = Units.Meters
                    .of(CatzTurret.Instance.getFieldToTurret().getDistance(FieldConstants.getHubLocation()));
            double angleRot = AimCalculations.calculateHoodBisectorAngle(dist.in(Units.Meters)) / (2 * Math.PI);
            return Setpoint.withMotionMagicSetpoint(angleRot);
        });
    }

    public Command applyFlywheelTuningSetpoint() {
        return Commands.defer(() -> {
            return CatzFlywheels.Instance.setpointCommand(
                    Setpoint.withVelocitySetpointVoltage((FlywheelConstants.SHOOTING_RPS_TUNABLE.get())));
        }, Set.of(CatzFlywheels.Instance));
    }

    // public Command turretManualCommand() {
    // return CatzTurret.Instance.followSetpointCommand(() -> {

    // });
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

    public Command alignToBackUpClimb() {
        return Commands.defer(() -> {
            Translation2d currentTranslation = CatzRobotTracker.Instance.getEstimatedPose().getTranslation();
            return new PIDDriveCmd(FieldConstants.getClimbAwayPosition(currentTranslation), true);
        }, Set.of(CatzDrivetrain.getInstance())).onlyIf(() -> isClimbMode || DriverStation.isAutonomous());
    }

    public Command alignToCloseClimb() {
        return Commands.defer(() -> {
            Translation2d currentTranslation = CatzRobotTracker.Instance.getEstimatedPose().getTranslation();
            return new PIDDriveCmd(FieldConstants.getClimbClosePosition(currentTranslation), true);
        }, Set.of(CatzDrivetrain.getInstance())).onlyIf(() -> isClimbMode || DriverStation.isAutonomous());
    }

    public Command autoClimbCommand() {
        return Commands.deadline(
                Commands.sequence(
                        cmdClimbReach(),
                        alignToBackUpClimb(),
                        alignToCloseClimb(),
                        stowIntake(),
                        cmdClimbStow()
                ),
                trackTower()
        ).onlyIf(() -> isClimbMode || DriverStation.isAutonomous());
    }

    public Command manualClimbUpCommand() {
        return cmdClimbReach().onlyIf(() -> isClimbMode);
    }

    public Command manualClimbDownCommand() {
        return cmdClimbStow().onlyIf(() -> isClimbMode);
    }

    public Command cmdClimbReach() {
        return CatzClimb.Instance.setpointCommand(ClimbConstants.REACH_SETPOINT);
    }

    public Command cmdClimbStow() {
        return CatzClimb.Instance.setpointCommand(ClimbConstants.STOW_SETPOINT);
    }
    public Command manualExtendClimb() {
        return CatzClimb.Instance.followSetpointCommand(() -> {
            if(Math.abs(RobotContainer.xboxTest.getLeftY()) < 0.07){
                return Setpoint.withVoltageSetpoint(0.0);
            }
            double input = -(RobotContainer.xboxTest.getLeftY()) * 12;
            Logger.recordOutput("Climb Xbox Voltage Input", input);
            return Setpoint.withVoltageSetpoint(input);
        });
    }

    public Command resetClimbPose(){
        return CatzClimb.Instance.setCurrentPositionCommand(Units.Rotations.of(0.0));
    }

    public Command enableClimbSoftLimit(){
        return Commands.runOnce(() -> {
            CatzClimb.Instance.setSoftLimitsEnabled(false, true);
        });
    }

    public Command disableClimbSoftLimit(){
        return Commands.runOnce(() -> {
            CatzClimb.Instance.setSoftLimitsEnabled(false, false);
        });
    }
}
