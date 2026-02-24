package frc.robot.CatzSubsystems;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
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
import frc.robot.Utilities.Setpoint;
import java.util.Set;

public class CatzSuperstructure {
    public static final CatzSuperstructure Instance = new CatzSuperstructure();
    private final CommandXboxController xboxTest = new CommandXboxController(1);
    private final CommandXboxController xboxDrv = new CommandXboxController(0);
    // NOTE use suppliers instead of creating two different objects

    private boolean isShootingAllowed = false; //TODO set to always true during auton

    private CatzSuperstructure() {
    }

    public Command turretTrackHubCommand() {
        return CatzTurret.Instance.followSetpointCommand(() -> AimCalculations.calculateHubTrackingSetpoint());
    }

    public Command hoodFlywheelStowCommand() {
        return Commands.parallel(
                CatzFlywheels.Instance.setpointCommand(FlywheelConstants.OFF_SETPOINT),
                CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOW_SETPOINT),
                setShootingAllowed(false));
    }

    public Command interpolateHoodAngle() {
        return CatzHood.Instance.followSetpointCommand(() -> {
            Pose2d turretPose = new Pose2d(CatzTurret.Instance.getFieldToTurret(), new Rotation2d());
            double distFromHub = FieldConstants.getHubLocation().getDistance(turretPose.getTranslation());

            return ShooterRegression.getHoodSetpoint(Units.Meters.of(distFromHub));
        });
    }

    public Command interpolateFlywheelSpeed() {
        return CatzFlywheels.Instance.followSetpointCommand(() -> {
            Translation2d turretPose = CatzTurret.Instance.getFieldToTurret();
            double distFromHub = FieldConstants.getHubLocation().getDistance(turretPose);

            return ShooterRegression.getShooterSetpointFromRegression(Units.Meters.of(distFromHub));
        });
    }

    /*/ public Command setHoodHomePosition() {
        return Commands.sequence(
            CatzHood.Instance.setpointCommand(Setpoint.withVoltageSetpoint(-1.5)),
            Commands.waitSeconds(0.2),
            Commands.waitUntil(() -> Math.abs(CatzHood.Instance.getVelocity().in(Units.DegreesPerSecond)) < 0.5),
            Commands.runOnce(() -> CatzHood.Instance.setCurrentPosition(Units.Degrees.of(0.0))),
            CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOP)
        );
    }
   */




    // public Command setHoodHomePosition() {
    //     return Commands.defer(() -> {
    //         // create debounce
    //         Debouncer stallDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
    //         return Commands.sequence(
    //             // apply neg volatge
    //             CatzHood.Instance.setpointCommand(Setpoint.withVoltageSetpoint(-1.5)),
    //             //  add frac of sec time so motor starm move
    //             Commands.waitSeconds(0.2),
    //             // wait for near0 volocity and .1 sec spike
    //             Commands.waitUntil(() -> {
    //                 boolean isStalling = Math.abs(CatzHood.Instance.getVelocity().in(Units.DegreesPerSecond)) < 0.5
    //                                   && CatzHood.Instance.getStatorCurrent() > 10.0;
    //                 return stallDebouncer.calculate(isStalling);
    //             }),
    //             // zero encoder where installed
    //             Commands.runOnce(() -> CatzHood.Instance.setCurrentPosition(Units.Degrees.of(0.0))),
    //             // stop motor
    //             CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOP)
    //         ).withTimeout(2.0); //failsafe
    //     }, Set.of(CatzHood.Instance));
    // }


    public Command interpolateShootingValues() {
        return Commands.run(() -> {
            Translation2d turretPose = CatzTurret.Instance.getFieldToTurret();
            Distance distFromHub = Units.Meters.of(FieldConstants.getHubLocation().getDistance(turretPose));
            CatzFlywheels.Instance.applySetpoint(ShooterRegression.getShooterSetpointFromRegression(distFromHub));
            CatzHood.Instance.applySetpoint(ShooterRegression.getHoodSetpoint(distFromHub));
        }, CatzFlywheels.Instance, CatzHood.Instance);
    }

    public Command prepareForShooting(){
        return Commands.parallel(
            interpolateShootingValues(),
            turretTrackHubCommand(),
            shootIfReady(),
            setShootingAllowed(false)
        );
    }

    public Command shootIfReady() {
        return Commands.run(() -> {
            boolean readyToShoot = AimCalculations.readyToShoot();
            if(readyToShoot && !isShootingAllowed){
                RobotContainer.rumbleDrv(1.0);
            }

            if (readyToShoot && isShootingAllowed) {
                CatzSpindexer.Instance.applySetpoint(SpindexerConstants.ON);
                CatzYdexer.Instance.applySetpoint(Setpoint.withVoltageSetpoint(YdexerConstants.SPEED.get()));
                RobotContainer.rumbleDrv(0.0);
            } else {
                CatzSpindexer.Instance.applySetpoint(SpindexerConstants.OFF);
                CatzYdexer.Instance.applySetpoint(YdexerConstants.OFF);
            }
        }, CatzSpindexer.Instance, CatzYdexer.Instance).finallyDo(() -> RobotContainer.rumbleDrv(0.0));
    }

    public Command startIndexers() {
        return Commands.parallel(
                // CatzSpindexer.Instance.setpointCommand(SpindexerConstants.ON),
                CatzYdexer.Instance.setpointCommand(() -> Setpoint.withVoltageSetpoint(YdexerConstants.SPEED.get())));
    }

    public Command stopIndexers() {
        return Commands.parallel(
                CatzSpindexer.Instance.setpointCommand(SpindexerConstants.OFF),
                CatzYdexer.Instance.setpointCommand(YdexerConstants.OFF));
    }

    public Command stopAllShooting() {
        return hoodFlywheelStowCommand().alongWith(stopIndexers()).alongWith(setShootingAllowed(false));
    }

    public Command setShootingAllowed(boolean val) {
        return Commands.runOnce(() -> isShootingAllowed = val);
    }

    public Command flywheelManualCommand() {
        return CatzFlywheels.Instance.followSetpointCommand(() -> {
            double input = (xboxDrv.getLeftY()) * 8;
            Logger.recordOutput("Xbox Voltage Input", input);
            return Setpoint.withVoltageSetpoint(input);
        });
    }

    public Command hoodManualCommand() {
        return CatzHood.Instance.followSetpointCommand(() -> {
            double input = -(xboxTest.getLeftY()) * 1;
            Logger.recordOutput("Xbox Voltage Input", input);
            return Setpoint.withVoltageSetpoint(input);
        });
    }

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

}
