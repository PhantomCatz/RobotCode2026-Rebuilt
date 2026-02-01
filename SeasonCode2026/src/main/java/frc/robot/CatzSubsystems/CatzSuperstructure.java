package frc.robot.CatzSubsystems;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
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
import frc.robot.Utilities.Setpoint;

public class CatzSuperstructure {
    public static final CatzSuperstructure Instance = new CatzSuperstructure();
    private final CommandXboxController xboxTest = new CommandXboxController(1);
    private final CommandXboxController xboxDrv = new CommandXboxController(0);
    // NOTE use suppliers instead of creating two different objects

    private CatzSuperstructure() {
    }

    public Command turretTrackCommand() {
        return CatzTurret.Instance.followSetpointCommand(() -> AimCalculations.calculateHubTrackingSetpoint());
    }

    public Command turret90Degrees(){
        return CatzTurret.Instance.setpointCommand(Setpoint.withMotionMagicSetpoint(0.25));
    }

    public Command turret90DegreesMinus(){
        return CatzTurret.Instance.setpointCommand(Setpoint.withMotionMagicSetpoint(-0.25));
    }

    public Command hoodFlywheelStowCommand() {
        return Commands.parallel(
                CatzFlywheels.Instance.setpointCommand(FlywheelConstants.OFF_SETPOINT),
                CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOW_SETPOINT));
    }

    public Command interpolateHoodAngle(){
        return CatzHood.Instance.followSetpointCommand(() -> {
            Pose2d turretPose = new Pose2d(CatzTurret.Instance.getFieldToTurret(), new Rotation2d());
            double distFromHub = FieldConstants.getHubLocation().getDistance(turretPose.getTranslation());

            return ShooterRegression.getHoodSetpoint(Units.Meters.of(distFromHub));
        });
    }

    public Command interpolateShooterSpeed(){
        return CatzFlywheels.Instance.followSetpointCommand(() -> {
            Pose2d turretPose = new Pose2d(CatzTurret.Instance.getFieldToTurret(), new Rotation2d());
            double distFromHub = FieldConstants.getHubLocation().getDistance(turretPose.getTranslation());

            return Setpoint.withVelocitySetpoint(ShooterRegression.getShooterRPSFromRegression(distFromHub));
        });
    }

    // public Command intakeDeployManualCommand(){
    // return CatzIntakeDeploy.Instance.followSetpointCommand(() -> {
    // double input = -xboxTest.getLeftY() * 3;
    // Logger.recordOutput("Xbox Input", input);
    // return Setpoint.withVoltageSetpoint(input);
    // });
    // }

    public Command startIndexers() {
        return Commands.parallel(
                CatzSpindexer.Instance.setpointCommand(SpindexerConstants.ON),
                CatzYdexer.Instance.setpointCommand(() -> Setpoint.withVoltageSetpoint(YdexerConstants.SPEED.get())));
    }

    public Command stopIndexers() {
        return Commands.parallel(
                CatzSpindexer.Instance.setpointCommand(SpindexerConstants.OFF),
                CatzYdexer.Instance.setpointCommand(YdexerConstants.OFF));
    }

    public Command stopAllShooting() {
        return hoodFlywheelStowCommand().alongWith(stopIndexers());
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
