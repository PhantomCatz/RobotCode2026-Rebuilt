package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzSuperstructure;
<<<<<<< HEAD
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzClimb.ClimbConstants;
=======
>>>>>>> 1e9341778a3b3de058c55a3f7049665d3172726b
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzHood.CatzHood;
import frc.robot.CatzSubsystems.CatzHood.HoodConstants;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntake;
import frc.robot.CatzSubsystems.CatzIntake.IntakeConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels;
import frc.robot.CatzSubsystems.CatzShooter.FlywheelConstants;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;

public class RobotContainer {
  private final CatzSuperstructure superstructure = CatzSuperstructure.Instance;
<<<<<<< HEAD
  private final CatzClimb climb = CatzClimb.Instance;
  private final CatzDrivetrain drivetrain = new CatzDrivetrain();
  private final CommandXboxController xboxDrv = new CommandXboxController(0);
=======

  public static final CommandXboxController xboxDrv = new CommandXboxController(0);
>>>>>>> 1e9341778a3b3de058c55a3f7049665d3172726b

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {
    CatzDrivetrain.Instance.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), () -> xboxDrv.getLeftY(), () -> xboxDrv.getRightX(), CatzDrivetrain.Instance));

    // xboxDrv.a().onTrue(superstructure.turretTrackCommand());
    // xboxDrv.x().onTrue(superstructure.turretStowCommand().alongWith(superstructure.hoodFlywheelStowCommand()));

    // xboxDrv.b().onTrue(superstructure.applyShooterSetpoint());
    // xboxDrv.y().onTrue(superstructure.flywheelManualCommand());
    // xboxDrv.a().onTrue(superstructure.)
    xboxDrv.y().onTrue(superstructure.hoodManualCommand());
    xboxDrv.a().onTrue(superstructure.applyFlywheelTuningSetpoint().alongWith(superstructure.applyHoodTuningSetpoint()));
    xboxDrv.b().onTrue(CatzFlywheels.Instance.setpointCommand(FlywheelConstants.OFF_SETPOINT));
    xboxDrv.x().onTrue(CatzHood.Instance.setCurrentPositionCommand(HoodConstants.HOOD_ZERO_POS));

    xboxDrv.leftBumper().onTrue(superstructure.hoodTestCommand());
    xboxDrv.rightBumper().onTrue(superstructure.applyShooterSetpoint());

  }

  public Command getAutonomousCommand() {
    return Commands.sequence(
      CatzIntake.Instance.followSetpointCommand(()->IntakeConstants.setpoint).withTimeout(2.0),
      Commands.waitSeconds(3),
      CatzIntake.Instance.followSetpointCommand(()->IntakeConstants.SETPOINT2)
    );
  }

   public Command getClimbCommand() {
    return Commands.sequence(
      CatzClimb.Instance.followSetpointCommand(()->ClimbConstants.Extend).withTimeout(2.0),
      Commands.waitSeconds(7),
      CatzClimb.Instance.followSetpointCommand(()->ClimbConstants.Stow).withTimeout(2.0)
    );
  }
}
