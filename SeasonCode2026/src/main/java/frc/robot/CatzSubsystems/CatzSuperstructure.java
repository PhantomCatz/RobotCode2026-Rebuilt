package frc.robot.CatzSubsystems;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzClimb.ClimbConstants;
import frc.robot.CatzSubsystems.CatzClimbTall.CatzClimbTall;
import frc.robot.CatzSubsystems.CatzClimbTall.ClimbConstantsTall;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzHood.CatzHood;
import frc.robot.CatzSubsystems.CatzHood.HoodConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels;
import frc.robot.CatzSubsystems.CatzShooter.FlywheelConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression;
import frc.robot.CatzSubsystems.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzTurret.TurretConstants;
import frc.robot.Utilities.InterpolatingDouble;
import frc.robot.Utilities.Setpoint;

public class CatzSuperstructure {
    public static final CatzSuperstructure Instance = new CatzSuperstructure();

    private CatzSuperstructure() {
    }

    public Command turretTrackCommand() {
        return CatzTurret.Instance.followSetpointCommand(() -> calculateHubTrackingSetpoint());
    }

    public Command turretStowCommand() {
        return CatzTurret.Instance.setpointCommand(TurretConstants.HOME_SETPOINT);
    }

    public Command hoodFlywheelStowCommand() {
        return Commands.parallel(
                CatzFlywheels.Instance.setpointCommand(FlywheelConstants.OFF_SETPOINT),
                CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOW_SETPOINT)
        );
    }

    public Command applyShooterSetpoint(){
        return CatzFlywheels.Instance.setpointCommand(FlywheelConstants.TEST_SETPOINT);
    }

    public Command flywheelManualCommand(){
        return CatzFlywheels.Instance.followSetpointCommand(() -> {
            double input = (RobotContainer.xboxDrv.getLeftY()) * 8;
            Logger.recordOutput("Xbox Voltage Input", input);
            return Setpoint.withVoltageSetpoint(input);
        });
    }

    public Command hoodManualCommand(){
        return CatzHood.Instance.followSetpointCommand(() -> {
            double input = -(RobotContainer.xboxDrv.getLeftY()) * 1;
            Logger.recordOutput("Xbox Voltage Input", input);
            return Setpoint.withVoltageSetpoint(input);
        });
    }

    public Command applyHoodTuningSetpoint(){
        return Commands.defer(() -> {
            Angle angle = Units.Degrees.of(HoodConstants.adjustableHoodAngle.get());

            return CatzHood.Instance.followSetpointCommand(() ->Setpoint.withMotionMagicSetpoint(angle));
        }, Set.of(CatzHood.Instance));
    }

    public Command applyFlywheelTuningSetpoint(){
        return Commands.defer(() -> {

            return CatzFlywheels.Instance.setpointCommand(Setpoint.withVelocitySetpointVoltage((FlywheelConstants.SHOOTING_RPS_TUNABLE.get())));
        }, Set.of(CatzFlywheels.Instance));
    }

    public Command extendClimbCommand(){
        return CatzClimb.Instance.setpointCommand(ClimbConstants.FULL_EXTEND);
    }

    public Command extendClimbTallCommand(){
        return CatzClimbTall.Instance.setpointCommand(ClimbConstantsTall.FULL_EXTEND);
    }

    public Command returnOriginalClimbCommand(){
        return CatzClimb.Instance.setpointCommand(ClimbConstants.HOME);
    }

    public Command returnOriginalClimbTallCommand(){
        return CatzClimbTall.Instance.setpointCommand(ClimbConstantsTall.HOME);
    }

    public Command tallShrinkShortRise(){
        return new ParallelCommandGroup(
            returnOriginalClimbTallCommand(),
            extendClimbCommand()
        );
    }
    public Command manualExtendCLimb() {
        return CatzClimb.Instance.followSetpointCommand(() -> {
            double input = (RobotContainer.xboxDrv.getLeftY()) * 1;
            Logger.recordOutput("Climb Xbox Voltage Input", input);
            return Setpoint.withVoltageSetpoint(input);
        });
    }

        public Command manualExtendClimbTall() {
        return CatzClimbTall.Instance.followSetpointCommand(() -> {
            double input = (RobotContainer.xboxDrv.getRightY()) * 1;
            Logger.recordOutput("Climb Xbox Voltage Input", input);
            return Setpoint.withVoltageSetpoint(input);
        });
    }



    public Command Climbing(){
        return new SequentialCommandGroup(
            extendClimbTallCommand(),
            tallShrinkShortRise(),
            extendClimbTallCommand(),
            tallShrinkShortRise(),
            extendClimbTallCommand(),
            tallShrinkShortRise()
        );
    }

    public Command hoodTestCommand(){
        return CatzHood.Instance.setpointCommand(HoodConstants.HOOD_TEST_SETPOINT);
    }

    public Command applyHoodSetpoint(){
        return CatzHood.Instance.setpointCommand(HoodConstants.HOOD_TEST_SETPOINT);
    }

    /**
     * Calculates the best turret angle setpoint to point to the hub
     * while respecting physical limits and minimizing movement
     */
    public Setpoint calculateHubTrackingSetpoint() {
        Pose2d robotPose = CatzRobotTracker.Instance.getEstimatedPose();
        Translation2d hubDirection = FieldConstants.HUB_LOCATION.minus(robotPose.getTranslation());
        double targetRads = hubDirection.getAngle().getRadians()
                - MathUtil.angleModulus(robotPose.getRotation().getRadians());

        return CatzTurret.Instance.calculateWrappedSetpoint(Angle.ofBaseUnits(targetRads, Units.Radians));
    }

    // interpolates distance to target for shooter setpoint along regression
    private double getShooterSetpointFromRegression(double range) {
        if (ShooterRegression.kUseFlywheelAutoAimPolynomial) {
            return ShooterRegression.kFlywheelAutoAimPolynomial.predict(range);
        } else {
            return ShooterRegression.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    // interpolates distance to target for hood setpoint along regression
    private double getHoodSetpointFromRegression(double range) {
        if (ShooterRegression.kUseHoodAutoAimPolynomial) {
            return ShooterRegression.kHoodAutoAimPolynomial.predict(range);
        } else {
            return ShooterRegression.kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    // public Command shootTuning(){
    // return
    // CatzFlywheels.Instance.setpointCommand(CatzShooter.Instance.getTunableSetpoint());
    // }

}
