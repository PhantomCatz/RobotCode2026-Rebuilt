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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzHood.CatzHood;
import frc.robot.CatzSubsystems.CatzHood.HoodConstants;
import frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer.CatzSpindexer;
import frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer.SpindexerConstants;
import frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer.CatzYdexer;
import frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer.YdexerConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels;
import frc.robot.CatzSubsystems.CatzShooter.FlywheelConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression;
import frc.robot.CatzSubsystems.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzTurret.TurretConstants;
import frc.robot.Utilities.Setpoint;

public class CatzSuperstructure {
    public static final CatzSuperstructure Instance = new CatzSuperstructure();
    private final CommandXboxController xboxTest = new CommandXboxController(1);
    private final CommandXboxController xboxDrv = new CommandXboxController(0);
    // NOTE use suppliers instead of creating two different objects

    private CatzSuperstructure() {
    }

    public Command turretTrackCommand() {
        return CatzTurret.Instance.followSetpointCommand(() -> calculateHubTrackingSetpoint());
    }

    public Command hoodFlywheelStowCommand() {
        return Commands.parallel(
                CatzFlywheels.Instance.setpointCommand(FlywheelConstants.OFF_SETPOINT)
        // CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOW_SETPOINT)
        );
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

    /**
     * Calculates the best turret angle setpoint to point to the hub
     * while respecting physical limits and minimizing movement
     */
    public Setpoint calculateHubTrackingSetpoint() {
        Pose2d fieldToRobot = CatzRobotTracker.Instance.getEstimatedPose();
        Pose2d fieldToTurret = fieldToRobot.transformBy(TurretConstants.TURRET_OFFSET);
        Translation2d hubDirection = FieldConstants.HUB_LOCATION.minus(fieldToTurret.getTranslation());

        double targetRads = hubDirection.getAngle().getRadians()
                - fieldToRobot.getRotation().getRadians();
        // if(DriverStation.getAlliance().get() == Alliance.Red){
        // targetRads -= Math.PI;
        // }
        double currentRads = CatzTurret.Instance.getPosition() * 2*Math.PI;
        double angleError = targetRads - currentRads;
        angleError = MathUtil.angleModulus(angleError);

        return CatzTurret.Instance.calculateWrappedSetpoint(Units.Radians.of(targetRads));
    }

    /**
     * Calculates the required velocity vector for the shooter flywheel/turret
     * to compensate for the indexer feed velocity.
     * * @return Translation2d where x/y represents the shooter's velocity vector
     * (in RPS) relative to the robot.
     */
    public Translation2d calculateShootingVector() {
        // 1. Get positions
        Pose2d fieldToRobot = CatzRobotTracker.Instance.getEstimatedPose();
        Pose2d fieldToTurret = fieldToRobot.transformBy(TurretConstants.TURRET_OFFSET);

        // 2. Calculate the vector pointing to the Hub
        Translation2d robotToHubVec = FieldConstants.HUB_LOCATION.minus(fieldToTurret.getTranslation());

        // 3. Determine the "Goal Magnitude" (Total Exit Velocity Needed)
        // The regression gives the Flywheel RPS needed for a straight shot.
        // The ACTUAL ball speed in that test was (Flywheel_RPS + Feeder_Speed).
        double regressionRPS = getShooterSetpointFromRegression(robotToHubVec.getNorm());
        double feederSpeedRPS = FlywheelConstants.VDEXER_FEED_COMPENSATION.getNorm();
        double totalRequiredSpeed = regressionRPS + feederSpeedRPS;

        // 4. Create the Goal Vector (Robot Relative)
        // This represents the path the ball must take through the air.
        // We take the direction to the hub (rotated to be robot-relative) and scale it
        // to the totalRequiredSpeed.
        Translation2d goalVector = robotToHubVec
                .rotateBy(fieldToRobot.getRotation().unaryMinus()) // Transform Field -> Robot
                .div(robotToHubVec.getNorm()) // Normalize to unit vector
                .times(totalRequiredSpeed); // Scale to the physics-true speed

        // 5. Get the Feeder Vector (Robot Relative)
        // This is the bias we need to cancel out.
        Translation2d feedVector = FlywheelConstants.VDEXER_FEED_COMPENSATION;

        // 6. Calculate the Shoot Vector
        // The Law of Velocities: V_goal = V_shooter + V_feed
        // Therefore: V_shooter = V_goal - V_feed
        Translation2d shootVector = goalVector.minus(feedVector);

        return shootVector;
    }

    // interpolates distance to target for shooter setpoint along regression
    private double getShooterSetpointFromRegression(double range) {
        if (ShooterRegression.kUseFlywheelAutoAimPolynomial) {
            return ShooterRegression.flywheelAutoAimPolynomial.predict(range);
        } else {
            return ShooterRegression.flywheelAutoAimMap.get(range);
        }
    }

}
