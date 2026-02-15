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

public class CatzSuperstructure {
    public static final CatzSuperstructure Instance = new CatzSuperstructure();
    private final CommandXboxController xboxTest = new CommandXboxController(1);
    private final CommandXboxController xboxDrv = new CommandXboxController(0);
    // NOTE use suppliers instead of creating two different objects

    private boolean isShootingAllowed = false; //TODO set to always true during auton

    private CatzSuperstructure() {
    }

    /**
     * Makes the turret track the hub.
     * 
     * @return A command to turn the turret to the hub
     */
    public Command turretTrackHubCommand() {
        return CatzTurret.Instance.followSetpointCommand(() -> AimCalculations.calculateHubTrackingSetpoint());
    }

    /**
     * Turns off the flywheels and and stows the hood.
     * 
     * @return A command to turn off the flywheels and stow the hood
     */
    public Command hoodFlywheelStowCommand() {
        return Commands.parallel(
                CatzFlywheels.Instance.setpointCommand(FlywheelConstants.OFF_SETPOINT),
                CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOW_SETPOINT),
                setShootingAllowed(false));
    }

    /**
     * Moves the hood to the desired position to be able to shoot the fuel into the hub.
     * 
     * @return A command to move the hood to target the hub
     */
    public Command interpolateHoodAngle() {
        return CatzHood.Instance.followSetpointCommand(() -> {
            Pose2d turretPose = new Pose2d(CatzTurret.Instance.getFieldToTurret(), new Rotation2d());
            double distFromHub = FieldConstants.getHubLocation().getDistance(turretPose.getTranslation());

            return ShooterRegression.getHoodSetpoint(Units.Meters.of(distFromHub));
        });
    }

    /**
     * Spins up the flywheels in order to shoot the fuel into the hub.
     * 
     * @return A command to spin up the flywheels to a speed to target the hub
     */
    public Command interpolateFlywheelSpeed() {
        return CatzFlywheels.Instance.followSetpointCommand(() -> {
            Translation2d turretPose = CatzTurret.Instance.getFieldToTurret();
            double distFromHub = FieldConstants.getHubLocation().getDistance(turretPose);

            return ShooterRegression.getShooterSetpointFromRegression(Units.Meters.of(distFromHub));
        });
    }

    /**
     * Targets hub with hood and flywheels.
     * 
     * @return A command to target the hub with the hood and flywheels
     */
    public Command interpolateShootingValues() {
        return Commands.run(() -> {
            Translation2d turretPose = CatzTurret.Instance.getFieldToTurret();
            Distance distFromHub = Units.Meters.of(FieldConstants.getHubLocation().getDistance(turretPose));
            CatzFlywheels.Instance.applySetpoint(ShooterRegression.getShooterSetpointFromRegression(distFromHub));
            CatzHood.Instance.applySetpoint(ShooterRegression.getHoodSetpoint(distFromHub));
        }, CatzFlywheels.Instance, CatzHood.Instance);
    }

    /**
     * Targets the hub with the turret and hood, and revs up flywheels. Starts shooting if ready.
     * 
     * @return A command to shoot at the hub
     */
    public Command prepareForShooting(){
        return Commands.parallel(
            interpolateShootingValues(),
            turretTrackHubCommand(),
            shootIfReady(),
            setShootingAllowed(false)
        );
    }

    /**
     * Shoots if ready, rumbles the controller if shooting is not allowed. 
     * 
     * @return A command to shoot into the hub if ready.
     */
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

    /**
     * Starts the Spindexer and Ydexer.
     * 
     * @return A command to turn on the Spin and Y dexers
     */
    public Command startIndexers() {
        return Commands.parallel(
                CatzSpindexer.Instance.setpointCommand(SpindexerConstants.ON),
                CatzYdexer.Instance.setpointCommand(() -> Setpoint.withVoltageSetpoint(YdexerConstants.SPEED.get())));
    }

    /**
     * Stops the Spindexer and Ydexer.
     * 
     * @return A command to turn off the Spin and Y dexers
     */
    public Command stopIndexers() {
        return Commands.parallel(
                CatzSpindexer.Instance.setpointCommand(SpindexerConstants.OFF),
                CatzYdexer.Instance.setpointCommand(YdexerConstants.OFF));
    }

    /**
     * Stops the Flywheels, Spindexer, and Ydexer, also returns the hood to it's home position.
     * 
     * @return A command to stop shooting
     */
    public Command stopAllShooting() {
        return hoodFlywheelStowCommand().alongWith(stopIndexers()).alongWith(setShootingAllowed(false));
    }

    /**
     * Sets if shooting fuel is allowed.
     * 
     * @param val if shooting is allowed
     * @return A command to set if shooting is allowed
     */
    public Command setShootingAllowed(boolean val) {
        return Commands.runOnce(() -> isShootingAllowed = val);
    }

    /**
     * Sets the flywheels to be controlled manually by the left joystick.
     * 
     * @return A command to enable manual control for the flywheels
     */
    public Command flywheelManualCommand() {
        return CatzFlywheels.Instance.followSetpointCommand(() -> {
            double input = (xboxTest.getLeftY()) * 8;
            Logger.recordOutput("Xbox Voltage Input", input);
            return Setpoint.withVoltageSetpoint(input);
        });
    }

    /**
     * Sets the hood to be controlled manually by the left joystick.
     * 
     * @return A command to enable manual control for the hood
     */
    public Command hoodManualCommand() {
        return CatzHood.Instance.followSetpointCommand(() -> {
            double input = -(xboxTest.getLeftY()) * 1;
            Logger.recordOutput("Xbox Voltage Input", input);
            return Setpoint.withVoltageSetpoint(input);
        });
    }

    /**
     * Applys the tuning setpoint to the hood.
     * 
     * @return A command to apply the tuning setpoint to the hood
     */
    public Command applyHoodTuningSetpoint() {
        return Commands.defer(() -> {
            Angle angle = Units.Degrees.of(HoodConstants.adjustableHoodAngle.get());

            return CatzHood.Instance.followSetpointCommand(() -> Setpoint.withMotionMagicSetpoint(angle));
        }, Set.of(CatzHood.Instance));
    }

    /**
     * Applys the tuning setpoint to the Flywheel.
     * 
     * @return A command to apply the tuning setpoint to the Flywheel
     */
    public Command applyFlywheelTuningSetpoint() {
        return Commands.defer(() -> {

            return CatzFlywheels.Instance.setpointCommand(
                    Setpoint.withVelocitySetpointVoltage((FlywheelConstants.SHOOTING_RPS_TUNABLE.get())));
        }, Set.of(CatzFlywheels.Instance));
    }

}
