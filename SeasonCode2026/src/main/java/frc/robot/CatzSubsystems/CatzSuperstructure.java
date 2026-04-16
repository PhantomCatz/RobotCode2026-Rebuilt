package frc.robot.CatzSubsystems;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CatzConstants;
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
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeDeploy.CatzIntakeDeploy;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeDeploy.IntakeDeployConstants;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels.CatzFlywheels;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels.FlywheelConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzHood.CatzHood;
import frc.robot.CatzSubsystems.CatzShooter.CatzHood.HoodConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.TurretConstants;
import frc.robot.Commands.DriveAndRobotOrientationCmds.PIDDriveCmd;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.Setpoint;

public class CatzSuperstructure {
    public static final CatzSuperstructure Instance = new CatzSuperstructure();

    private CatzSuperstructure() {
        CatzConstants.autoFactory = new AutoFactory(
                                                  CatzRobotTracker.getInstance()::getEstimatedPose,
                                                  CatzRobotTracker.getInstance()::resetPose,
                                                  CatzDrivetrain.getInstance()::followChoreoTrajectoryExecute,
                                                  true,
                                                  CatzDrivetrain.getInstance()
                                                ); //it is apparently a good idea to initialize these variables not statically because there can be race conditions

    }
    
    //-------------------- 
    //    Shooter
    //--------------------
    public Command FlyWheelOn(){
        return CatzFlywheels.Instance.setpointCommand(FlywheelConstants.TEST_SETPOINT);
    }

    public Command FlyWheelOff(){
        return CatzFlywheels.Instance.setpointCommand(FlywheelConstants.OFF_SETPOINT);
    }

    public Command HubScore(){
        return null;
    }

    public Command Hoard(){
        return null;
    }

    //-------------------- 
    //    Indexer
    //--------------------
    public Command SpindexerOn(){
        return CatzSpindexer.Instance.setpointCommand(SpindexerConstants.ON);
    }

    public Command YdexerOn(){
        return CatzYdexer.Instance.setpointCommand(YdexerConstants.ON);
    }

    public Command SpindexerOff(){
        return CatzSpindexer.Instance.setpointCommand(SpindexerConstants.OFF);
    }

    public Command YdexerOff(){
        return CatzYdexer.Instance.setpointCommand(YdexerConstants.OFF);
    }

    public Command ShooterToggle(){
        return null;
    }
}
