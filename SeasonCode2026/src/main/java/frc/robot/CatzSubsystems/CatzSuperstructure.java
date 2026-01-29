package frc.robot.CatzSubsystems;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
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
import frc.robot.CatzSubsystems.CatzIntakeDeploy.CatzIntakeDeploy;
import frc.robot.CatzSubsystems.CatzIntakeDeploy.IntakeDeployConstants;
import frc.robot.CatzSubsystems.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntakeRoller.IntakeRollerConstants;
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
    //NOTE use suppliers instead of creating two different objects


    private CatzSuperstructure() {
    }

    public Command turretTrackCommand() {
        return CatzTurret.Instance.followSetpointCommand(() -> calculateHubTrackingSetpoint());
    }

    public Command turretTrackCommandNoOffset() {
        return CatzTurret.Instance.followSetpointCommand(() -> calculateHubTrackingSetpointNoOffset());

    }

    public Command turretStowCommand() {
        return CatzTurret.Instance.setpointCommand(TurretConstants.HOME_SETPOINT);
    }

    public Command turretManualTrackCommand() {
        // return CatzTurret.Instance.setpointCommand(Setpoint.withDutyCycleSetpoint(0.1));
        // return CatzTurret.Instance.setpointCommand(Setpoint.withPositionSetpoint(Units.Degrees.of(90.0)));
        // return CatzTurret.Instance.setpointCommand(Setpoint.withVelocitySetpoint(1.0));
        return CatzTurret.Instance.followSetpointCommand(() -> {

                double input = xboxTest.getLeftY();

                return Setpoint.withVoltageSetpoint(input);
            }
        );
        // return CatzTurret.Instance.followSetpointCommand(() -> {
        //     double input = xboxTest.getLeftY() * 5;
        //     Logger.recordOutput("Xbox Inputted", input);
        //     return Setpoint.withVoltageSetpoint(input);
        // });
    }

    public Command hoodFlywheelStowCommand() {
        return Commands.parallel(
                CatzFlywheels.Instance.setpointCommand(FlywheelConstants.OFF_SETPOINT),
                CatzHood.Instance.setpointCommand(HoodConstants.HOOD_STOW_SETPOINT)
        );
    }

    // public Command intakeDeployManualCommand(){
    //     return CatzIntakeDeploy.Instance.followSetpointCommand(() -> {
    //         double input = -xboxTest.getLeftY() * 3;
    //         Logger.recordOutput("Xbox Input", input);
    //         return Setpoint.withVoltageSetpoint(input);
    //     });
    // }

    public Command startIndexers(){
        return Commands.parallel(
            CatzSpindexer.Instance.setpointCommand(SpindexerConstants.ON),
            CatzYdexer.Instance.setpointCommand(YdexerConstants.ON)
        );
    }

    public Command stopIndexers(){
        return Commands.parallel(
            CatzSpindexer.Instance.setpointCommand(SpindexerConstants.OFF),
            CatzYdexer.Instance.setpointCommand(YdexerConstants.OFF)
        );
    }

    public Command stopAllShooting(){
        return hoodFlywheelStowCommand().alongWith(stopIndexers());
    }

    public Command flywheelManualCommand(){
        return CatzFlywheels.Instance.followSetpointCommand(() -> {
            double input = (xboxDrv.getLeftY()) * 8;
            Logger.recordOutput("Xbox Voltage Input", input);
            return Setpoint.withVoltageSetpoint(input);
        });
    }

    public Command hoodManualCommand(){
        return CatzHood.Instance.followSetpointCommand(() -> {
            double input = -(xboxTest.getLeftY()) * 1;
            Logger.recordOutput("Xbox Voltage Input", input);
            return Setpoint.withVoltageSetpoint(input);
        });
    }


    public Command IntakeOn(){
        return CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.H_SETPOINT);
    }

    public Command IntakeOff(){
        return CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT);
    }

    public Command SlapDown() {
        return CatzIntakeDeploy.Instance.setpointCommand(IntakeDeployConstants.HoldDown);
    }

    /**
     * Calculates the best turret angle setpoint to point to the hub
     * while respecting physical limits and minimizing movement
     */
    public Setpoint calculateHubTrackingSetpoint() {
        Pose2d fieldToRobot = CatzRobotTracker.Instance.getEstimatedPose();
        Pose2d fieldToTurret = fieldToRobot.transformBy(TurretConstants.TURRET_OFFSET);
        Translation2d hubDirection = FieldConstants.HUB_LOCATION.minus(fieldToTurret.getTranslation());
        Logger.recordOutput("Hub Location", FieldConstants.HUB_LOCATION);
        Logger.recordOutput("Turret Location", fieldToTurret);
        Logger.recordOutput("Hub Direction", hubDirection);
        double targetRads = hubDirection.getAngle().getRadians()
                - fieldToRobot.getRotation().getRadians();
        // if(DriverStation.getAlliance().get() == Alliance.Red){
        //     targetRads -= Math.PI;
        // }
        double currentRads = CatzTurret.Instance.getPosition() * 2*Math.PI;
        // Logger.recordOutput("Turret Current Location", fieldToTurret.rotateBy(Rotation2d.fromRadians(currentRads)));
        // Logger.recordOutput("Turret Target Location", fieldToTurret.rotateBy(Rotation2d.fromRadians(targetRads)));
        double angleError = targetRads - currentRads;
        angleError = MathUtil.angleModulus(angleError);
        // Logger.recordOutput("Turret Calculate Commanded Setpoint", targetRads / (2*Math.PI));
        return CatzTurret.Instance.calculateWrappedSetpoint(Units.Radians.of(targetRads)); //TODO PUT THE WRAPPING BACK
        // return Setpoint.withMotionMagicSetpoint(Units.Radians.of(currentRads+angleError)); THIS IS THE NO WRAP
    }

    public Setpoint calculateHubTrackingSetpointNoOffset() {
        Pose2d fieldToRobot = CatzRobotTracker.Instance.getEstimatedPose();
        Translation2d hubDirection = FieldConstants.HUB_LOCATION.minus(fieldToRobot.getTranslation());
        double targetRads = hubDirection.getAngle().getRadians()
                - MathUtil.angleModulus(fieldToRobot.getRotation().getRadians());
        Logger.recordOutput("Turret Calculate Commanded Setpoint", targetRads / (2*Math.PI));
        return CatzTurret.Instance.calculateWrappedSetpoint(Units.Radians.of(targetRads));
    }

    // interpolates distance to target for shooter setpoint along regression
    private double getShooterSetpointFromRegression(double range) {
        if (ShooterRegression.kUseFlywheelAutoAimPolynomial) {
            return ShooterRegression.flywheelAutoAimPolynomial.predict(range);
        } else {
            return ShooterRegression.flywheelAutoAimMap.get(range);
        }
    }

    // public Command shootTuning(){
    // return
    // CatzFlywheels.Instance.setpointCommand(CatzShooter.Instance.getTunableSetpoint());
    // }

}
