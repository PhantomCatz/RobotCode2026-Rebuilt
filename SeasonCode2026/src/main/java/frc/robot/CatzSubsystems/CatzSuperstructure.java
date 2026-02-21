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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimbClaw.CatzClimbClaw;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimbClaw.ClimbConstantsClaw;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimbElevator.CatzClimbElevator;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimbElevator.ClimbConstantsElevator;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels.CatzFlywheels;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels.FlywheelConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzHood.CatzHood;
import frc.robot.CatzSubsystems.CatzShooter.CatzHood.HoodConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.TurretConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression;
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


    public WaitUntilCommand elevatorExtendThreshold() {
        return new WaitUntilCommand(() -> CatzClimbElevator.Instance.getLatencyCompensatedPosition() >= ClimbConstantsElevator.CLAW_EXTEND_THRESHOLD);
    }

    public WaitUntilCommand elevatorRetractThreshold() {
        return new WaitUntilCommand(() -> CatzClimbElevator.Instance.getLatencyCompensatedPosition() >= ClimbConstantsElevator.CLAW_RETRACT_THRESHOLD);
    }

    public Command extendClimbElevatorCommand(){
        return CatzClimbElevator.Instance.setpointCommand(ClimbConstantsElevator.FULL_EXTEND);
    }

    public Command extendClimbClawCommand(){
        return CatzClimbClaw.Instance.setpointCommand(ClimbConstantsClaw.FULL_EXTEND);
    }

    public Command retractClimbElevatorCommand(){
        return CatzClimbElevator.Instance.setpointCommand(ClimbConstantsElevator.HOME);
    }

     public Command retractClimbClawCommand(){
        return CatzClimbClaw.Instance.setpointCommand(ClimbConstantsClaw.HOME);
    }

    public ParallelCommandGroup extendFullClimb() {
        return new ParallelCommandGroup(
            extendClimbElevatorCommand(),
            new SequentialCommandGroup(
                elevatorExtendThreshold(),
                extendClimbClawCommand()
            )
        );
    }

    //passive hooks from elevator lock onto to tower, so no need for multiple claws
    public ParallelCommandGroup RetractFullClimb() {
        return new ParallelCommandGroup(
            retractClimbElevatorCommand(),
            new SequentialCommandGroup(
                elevatorRetractThreshold(),
                retractClimbClawCommand()
            )
        );
    }

    public SequentialCommandGroup levelOneClimb() {
        return new SequentialCommandGroup(
            extendFullClimb(),
            RetractFullClimb()
        );
    }

    public SequentialCommandGroup levelTwoClimb() {
        return new SequentialCommandGroup(
            levelOneClimb(),
            levelOneClimb()
        );
    }

    public SequentialCommandGroup levelThreeClimb() {
        return new SequentialCommandGroup(
            levelTwoClimb(),
            levelOneClimb()
        );
    }

    public Command manualExtendClimb() {
        return CatzClimbElevator.Instance.followSetpointCommand(() -> {
            double input = (RobotContainer.xboxDrv.getLeftY()) * 2;
            Logger.recordOutput("Climb Xbox Voltage Input", input);
            return Setpoint.withVoltageSetpoint(input);
        });
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
        Translation2d hubDirection = FieldConstants.getHubLocation().minus(robotPose.getTranslation());
        double targetRads = hubDirection.getAngle().getRadians()
                - MathUtil.angleModulus(robotPose.getRotation().getRadians());

        return CatzTurret.Instance.calculateWrappedSetpoint(Angle.ofBaseUnits(targetRads, Units.Radians));
    }

    // interpolates distance to target for shooter setpoint along regression
    private double getShooterSetpointFromRegression(double range) {
        if (ShooterRegression.kUseFlywheelAutoAimPolynomial) {
            return ShooterRegression.flywheelAutoAimPolynomial.predict(range);
        } else {
            return 0.0;
        }
    }


    // public Command shootTuning(){
    // return
    // CatzFlywheels.Instance.setpointCommand(CatzShooter.Instance.getTunableSetpoint());
    // }

}
