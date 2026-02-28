package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzClimbElevator.CatzClimbElevator;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression;
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightSubsystem;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.DoublePressTracker;

public class RobotContainer {
  private final CatzSuperstructure superstructure = CatzSuperstructure.Instance;
  private final CatzClimbElevator climb = CatzClimbElevator.Instance;
  private final CatzDrivetrain drivetrain = new CatzDrivetrain();
  public static final CommandXboxController xboxDrv = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();

    var turret = CatzTurret.Instance;
    var tracker = CatzRobotTracker.Instance;
    var vision = LimelightSubsystem.Instance;
    var regression = ShooterRegression.TUNABLE_HOOD_ANGLE_MIN;
  }

  private void configureBindings() {
    // CatzDrivetrain.Instance.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), () -> xboxDrv.getLeftY(),
    //     () -> xboxDrv.getRightX(), CatzDrivetrain.Instance));
    // CatzTurret.Instance.setDefaultCommand(
    // superstructure.turretManualTrackCommand()
    DoublePressTracker.createTrigger(xboxDrv.back()).onTrue(new InstantCommand(() -> {
      if (AllianceFlipUtil.shouldFlip()) {
        CatzRobotTracker.Instance
            .resetPose(new Pose2d(CatzRobotTracker.Instance.getEstimatedPose().getTranslation(), Rotation2d.k180deg));
      } else {
        CatzRobotTracker.Instance
            .resetPose(new Pose2d(CatzRobotTracker.Instance.getEstimatedPose().getTranslation(), new Rotation2d()));
      }
    }));
    // );

    // ----------------------Shooting-----------------------
    // xboxDrv.leftBumper().onTrue(superstructure.prepareForShooting());
    // xboxDrv.leftBumper().onFalse(superstructure.setShootingAllowed(true));
    // xboxDrv.x().onTrue(superstructure.stopAllShooting());


    // xboxDrv.b().onTrue(superstructure.applyShooterSetpoint());
    // xboxDrv.y().onTrue(superstructure.flywheelManualCommand());
    // xboxDrv.a().onTrue(superstructure.)
    // xboxDrv.y().onTrue(superstructure.hoodManualCommand());
    // xboxDrv.a().onTrue(superstructure.applyFlywheelTuningSetpoint().alongWith(superstructure.applyHoodTuningSetpoint()));
    // xboxDrv.b().onTrue(CatzFlywheels.Instance.setpointCommand(FlywheelConstants.OFF_SETPOINT));
    // xboxDrv.x().onTrue(CatzHood.Instance.setCurrentPositionCommand(HoodConstants.HOOD_ZERO_POS));

    // xboxDrv.leftBumper().onTrue(superstructure.hoodTestCommand());
    // xboxDrv.rightBumper().onTrue(superstructure.applyShooterSetpoint());

    xboxDrv.a().onTrue(superstructure.manualExtendClimb());
      // xboxTest.a().onTrue(superstructure.startIndexers());
    // xboxTest.x().onTrue(superstructure.stopAllShooting());

  }

  public static void rumbleDrv(double val) {
    xboxDrv.setRumble(RumbleType.kBothRumble, val);
  }

  //  public Command getClimbCommand() {
  //   return Commands.sequence(
  //     CatzClimb.Instance.followSetpointCommand(()->ClimbConstants.Extend).withTimeout(2.0),
  //     Commands.waitSeconds(7),
  //     CatzClimb.Instance.followSetpointCommand(()->ClimbConstants.Stow).withTimeout(2.0)
  //   );
  // }
}
