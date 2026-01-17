package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntake;
import frc.robot.CatzSubsystems.CatzIntake.IntakeConstants;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels;
import frc.robot.Utilities.Setpoint;

public class RobotContainer {
  private final CatzSuperstructure superstructure = CatzSuperstructure.Instance;

  private final CommandXboxController xboxDrv = new CommandXboxController(0);

  public RobotContainer() {
    System.out.println("Drivetrain Initializing" + CatzDrivetrain.Instance.getName());
    System.out.println("Turret Initializing" + CatzTurret.Instance.getName());
    System.out.println("Hood Initializing" + CatzHood.Instance.getName());
    System.out.println("Intake Initializing" + CatzIntake.Instance.getName());
    System.out.println("Initializing Climb" + CatzClimb.Instance.getName());

    configureBindings();
  }

  private void configureBindings() {
    // CatzDrivetrain.Instance.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), () -> xboxDrv.getLeftY(), () -> xboxDrv.getRightX(), CatzDrivetrain.Instance));

    xboxDrv.a().onTrue(superstructure.turretTrackCommand());
    xboxDrv.x().onTrue(superstructure.turretStowCommand().alongWith(superstructure.hoodFlywheelStowCommand()));

    xboxDrv.leftBumper().onTrue(superstructure.shootingTuneCommand());


    xboxDrv.b().onTrue(CatzFlywheels.Instance.setpointCommand(Setpoint.withVelocitySetpointVoltage(100.0)));
  }

  public Command getAutonomousCommand() {
    return Commands.sequence(
      CatzIntake.Instance.followSetpointCommand(()->IntakeConstants.setpoint).withTimeout(2.0),
      Commands.waitSeconds(3),
      CatzIntake.Instance.followSetpointCommand(()->IntakeConstants.SETPOINT2)
    );
  }
}
