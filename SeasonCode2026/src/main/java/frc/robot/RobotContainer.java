package frc.robot;


import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightSubsystem;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.DoublePressTracker;

public class RobotContainer {
  private final CatzSuperstructure superstructure = CatzSuperstructure.Instance;

  public static final CommandXboxController xboxDrv = new CommandXboxController(0);
  public static final CommandXboxController xboxAux = new CommandXboxController(1);

  public RobotContainer() {
    configureBindings();

    var turret = CatzTurret.Instance;
    var tracker = CatzRobotTracker.Instance;
    var vision = LimelightSubsystem.Instance;
    var climb = CatzClimb.Instance;
    var superstructure = CatzSuperstructure.Instance;
    System.out.println(superstructure);

  }

  private void configureBindings() {
    xboxDrv.rightBumper().onTrue(CatzSuperstructure.Instance.scoreHub);
    xboxDrv.leftBumper().onTrue(CatzSuperstructure.Instance.Hoard);
  }

  public static void rumbleDrv(double val) {
    xboxDrv.setRumble(RumbleType.kBothRumble, val);
  }
}
