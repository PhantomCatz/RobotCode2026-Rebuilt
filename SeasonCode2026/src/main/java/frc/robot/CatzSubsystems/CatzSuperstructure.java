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
import frc.robot.Utilities.Setpoint;

public class CatzSuperstructure {
    public static final CatzSuperstructure Instance = new CatzSuperstructure();

    private boolean isCloseCornerHoarding = true;

    private CatzSuperstructure() {}

    private Translation2d getTargetLocation(RegressionMode mode) {
        if (mode == RegressionMode.HUB) {
            return FieldConstants.getHubLocation();
        } else {
            return AimCalculations.getCornerHoardingTarget(isCloseCornerHoarding);
        }
    }

    private RegressionMode getSpecificMode(RegressionMode mode) {
        if (mode != RegressionMode.HUB) {
            return isCloseCornerHoarding ? RegressionMode.CLOSE_HOARD : RegressionMode.OPP_HOARD;
        }
        return mode;
    }

    public Command trackTarget(RegressionMode mode) {
        return Commands.run(() -> {
            Translation2d targetLoc = getTargetLocation(mode);
            CatzTurret.Instance.applySetpoint(AimCalculations.calculateTurretTrackingSetpoint(targetLoc));
        }, CatzTurret.Instance);
    }

    public Command rampUpFlywheels(RegressionMode mode) {
        return Commands.run(() -> {
            Translation2d targetLoc = getTargetLocation(mode);
            Translation2d turretPos = CatzTurret.Instance.getFieldToTurret();
            Distance dist = Units.Meters.of(targetLoc.getDistance(turretPos));

            RegressionMode specificMode = getSpecificMode(mode);

            CatzFlywheels.Instance.applySetpoint(ShooterRegression.getShooterSetpoint(dist, specificMode));
        }, CatzFlywheels.Instance);
    }

    private Command aimHood(RegressionMode mode) {
        return Commands.run(() -> {
            Translation2d targetLoc = (mode == RegressionMode.HUB) ?
                FieldConstants.getHubLocation() :
                AimCalculations.getCornerHoardingTarget(isCloseCornerHoarding);

            Distance dist = Units.Meters.of(targetLoc.getDistance(CatzTurret.Instance.getFieldToTurret()));

            RegressionMode specificMode = mode;
            if (mode != RegressionMode.HUB) {
                specificMode = isCloseCornerHoarding ? RegressionMode.CLOSE_HOARD : RegressionMode.OPP_HOARD;
            }

            CatzHood.Instance.applySetpoint(ShooterRegression.getHoodSetpoint(dist, specificMode));
        }, CatzHood.Instance);
    }

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

    public Command jiggleIntakeCommand() {
        return Commands.run(() -> {
            double time = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

            // Evaluates to 15.0 when sine is positive and 0.0 when negative
            double angleDeg = Math.sin(time * 2) > 0 ? 15.0 : 0.0;

            CatzIntakeDeploy.Instance.applySetpoint(
                Setpoint.withMotionMagicSetpoint(Units.Rotation.of(angleDeg / 360.0))
            );
        }, CatzIntakeDeploy.Instance);
    }

    public Command cmdFullStop() {
        return Commands.parallel(
            CatzFlywheels.Instance.setpointCommand(FlywheelConstants.OFF_SETPOINT),
            CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOW_SETPOINT),
            CatzSpindexer.Instance.setpointCommand(SpindexerConstants.OFF),
            CatzYdexer.Instance.setpointCommand(YdexerConstants.OFF)
        );
    }

    public Command cmdHoardShoot() {
        return Commands.parallel(
            trackTarget(RegressionMode.CLOSE_HOARD),
            rampUpFlywheels(RegressionMode.CLOSE_HOARD),
            aimHood(RegressionMode.CLOSE_HOARD),
            runFeeder(),
            jiggleIntakeCommand()
        );
    }

    public Command cmdHoardStandby() {
        return Commands.parallel(
            trackTarget(RegressionMode.CLOSE_HOARD),
            rampUpFlywheels(RegressionMode.CLOSE_HOARD),
            CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOW_SETPOINT),
            CatzSpindexer.Instance.setpointCommand(SpindexerConstants.OFF),
            CatzYdexer.Instance.setpointCommand(YdexerConstants.OFF),
            CatzIntakeDeploy.Instance.setpointCommand(IntakeDeployConstants.STOW)
        );
    }

    public Command cmdHubShoot() {
        return Commands.parallel(
            trackTarget(RegressionMode.HUB),
            rampUpFlywheels(RegressionMode.HUB),
            aimHood(RegressionMode.HUB),
            runFeeder(),
            jiggleIntakeCommand()
        );
    }

    public Command cmdHubStandby() {
        return Commands.parallel(
            trackTarget(RegressionMode.HUB),
            rampUpFlywheels(RegressionMode.HUB),
            CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOW_SETPOINT),
            CatzSpindexer.Instance.setpointCommand(SpindexerConstants.OFF),
            CatzYdexer.Instance.setpointCommand(YdexerConstants.OFF),
            CatzIntakeDeploy.Instance.setpointCommand(IntakeDeployConstants.STOW)
        );
    }

    public Command toggleHoardLocation() {
        return Commands.runOnce(() -> {
            isCloseCornerHoarding = !isCloseCornerHoarding;
        });
    }

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
            }
        }, CatzIntakeRoller.Instance);
    }

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

    // public Command setHoodHomePosition() {
    //     return Commands.defer(() -> {
    //         Debouncer stallDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    //         return Commands.sequence(
    //             CatzHood.Instance.setpointCommand(Setpoint.withVoltageSetpoint(-1.5)),
    //             Commands.waitSeconds(0.2),
    //             Commands.waitUntil(() -> {
    //                 boolean isStalling = Math.abs(CatzHood.Instance.getVelocity().in(Units.DegreesPerSecond)) < 0.5
    //                                   && CatzHood.Instance.getStatorCurrent() > 10.0;

    //                 return stallDebouncer.calculate(isStalling);
    //             }),
    //             Commands.runOnce(() -> CatzHood.Instance.setCurrentPosition(Units.Degrees.of(0.0))),
    //             CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOP)
    //         ).withTimeout(2.0);
    //     }, Set.of(CatzHood.Instance));
    // }

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

    public Command turret30Deg() {
        return CatzTurret.Instance.setpointCommand(Setpoint.withMotionMagicSetpoint(Units.Degrees.of(30.0)));
    }
    public Command turretMinus30Deg() {
        return CatzTurret.Instance.setpointCommand(Setpoint.withMotionMagicSetpoint(Units.Degrees.of(-30.0)));
    }

    public Command turretTrackHubCommand() {
        return CatzTurret.Instance.followSetpointCommand(() -> AimCalculations.calculateHubTrackingSetpoint());
    }

    public Command turretTrackCornerCommand(){
        return CatzTurret.Instance.followSetpointCommand(() -> AimCalculations.calculateCornerTrackingSetpoint());
    }
}
