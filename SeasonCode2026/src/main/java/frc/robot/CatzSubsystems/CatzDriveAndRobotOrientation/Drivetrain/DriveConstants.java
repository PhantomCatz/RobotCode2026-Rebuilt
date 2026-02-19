package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.CatzConstants;
import frc.robot.Utilities.HolonomicDriveController;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.ModuleLimits;
import lombok.Builder;

public class DriveConstants {
  // ---------------------------------------------------------------------------------------------------------------
  // Disabled flag for testing
  // ---------------------------------------------------------------------------------------------------------------
  public static final boolean IS_DRIVE_DISABLED = false; //bruh
  public static final boolean IS_FOC = true;

  // ---------------------------------------------------------------------------------------------------------------
  // Module organizations
  // ---------------------------------------------------------------------------------------------------------------
  public static final String[] MODULE_NAMES = new String[] {"FR", "BR", "BL", "FL"};
  public static final int INDEX_FR = 0;
  public static final int INDEX_BR = 1;
  public static final int INDEX_BL = 2;
  public static final int INDEX_FL = 3;

  public static final int GYRO_ID = 10;

  // ---------------------------------------------------------------------------------------------------------------
  // Drive Subsytem Config info
  // ---------------------------------------------------------------------------------------------------------------


  public static final DriveConfig DRIVE_CONFIG =
    DriveConfig.builder()
        .wheelRadius(Units.inchesToMeters(1.7))
        .robotLengthX(Units.inchesToMeters(28))
        .robotWidthY(Units.inchesToMeters(28))
        .bumperWidthX(Units.inchesToMeters(32))
        .bumperWidthY(Units.inchesToMeters(32))
        .maxLinearVelocity(4.3)
        .maxLinearAcceleration(22)
        .maxAngularVelocity(Units.degreesToRadians(540))
        .maxAngularAcceleration(Units.degreesToRadians(720))
        .build();

  public static final DriveConfig TRAJECTORY_CONFIG =
    DriveConfig.builder()
      .maxLinearVelocity(4.3)
      .maxLinearAcceleration(5.5)
      .maxAngularVelocity(Units.degreesToRadians(540))
      .maxAngularAcceleration(Units.degreesToRadians(720))
      .build();
  private static final LoggedTunableNumber accLimit = new LoggedTunableNumber("accLimit", 22.0);

  public static final ModuleLimits MOVE_WHILE_SHOOT_LIMITS = new ModuleLimits(
        DriveConstants.DRIVE_CONFIG.maxLinearVelocity(),
        accLimit.get(),
        DriveConstants.DRIVE_CONFIG.maxAngularVelocity());

  public static final ModuleLimits DRIVE_LIMITS = new ModuleLimits(
    DriveConstants.DRIVE_CONFIG.maxLinearVelocity(),
    DriveConstants.DRIVE_CONFIG.maxLinearAcceleration(),
    DriveConstants.DRIVE_CONFIG.maxAngularVelocity());

  public static final ModuleGainsAndRatios MODULE_GAINS_AND_RATIOS =
      switch (CatzConstants.getRobotType()) {
        case SN1, SN2, BUBBLES, SN_MANTA ->
            new ModuleGainsAndRatios(
                5.0,
                0.45,
                1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp, // A/(N*m)
                6.0,
                0.0,
                1.0,
                0.00,
                Mk4iReductions.L2_16t.reduction,
                Mk4iReductions.steer.reduction);
        case SN_TEST ->
            new ModuleGainsAndRatios(
                0.014,
                0.134,
                0.0,
                0.1,
                0.0,
                1.0,
                0.0,
                Mk4iReductions.L2_16t.reduction,
                Mk4iReductions.steer.reduction);

          default ->
             new ModuleGainsAndRatios(
                0.014,
                0.134,
                0.0,
                0.1,
                0.0,
                1.0,
                0.0,
                Mk4iReductions.L2_16t.reduction,
                Mk4iReductions.steer.reduction);
      };
  // -------------------------------------------------------------------------------
  // Odometry Constants
  // -------------------------------------------------------------------------------

  public static final double GYRO_UPDATE_FREQUENCY =
      switch (CatzConstants.getRobotType()) {
        case SN_TEST -> 50.0;
        case SN2, SN1, BUBBLES -> 100.0;
        //case SN2 -> 250.0;
        default -> 100.0;
      };

  // ---------------------------------------------------------------------------------------------------------------------
  // Logged Tunable PIDF values for swerve modules
  // ---------------------------------------------------------------------------------------------------------------------
  public static final LoggedTunableNumber drivekP = new LoggedTunableNumber("Drive/Module/DrivekP", MODULE_GAINS_AND_RATIOS.drivekP());
  public static final LoggedTunableNumber drivekD = new LoggedTunableNumber("Drive/Module/DrivekD", MODULE_GAINS_AND_RATIOS.drivekD());
  public static final LoggedTunableNumber drivekS = new LoggedTunableNumber("Drive/Module/DrivekS", MODULE_GAINS_AND_RATIOS.driveFFkS());
  public static final LoggedTunableNumber drivekV = new LoggedTunableNumber("Drive/Module/DrivekV", MODULE_GAINS_AND_RATIOS.driveFFkV());
  public static final LoggedTunableNumber steerkP = new LoggedTunableNumber("Drive/Module/steerkP", MODULE_GAINS_AND_RATIOS.steerkP());
  public static final LoggedTunableNumber steerkD = new LoggedTunableNumber("Drive/Module/steerkD", MODULE_GAINS_AND_RATIOS.steerkD());

  public static final ModuleIDs[] MODULE_CONFIGS = new ModuleIDs[4];
  static{
    switch(CatzConstants.getRobotType()){
        case SN_MANTA:
            MODULE_CONFIGS[INDEX_FR] = new ModuleIDs(1, 2, 11, 2.54248, false);
            MODULE_CONFIGS[INDEX_BR] = new ModuleIDs(3, 4, 12, -2.866211, false);
            MODULE_CONFIGS[INDEX_BL] = new ModuleIDs(5, 6, 13, -0.988281, false);
            MODULE_CONFIGS[INDEX_FL] = new ModuleIDs(7, 8, 14, -0.416016, false);
        break;

        case SN1:
            MODULE_CONFIGS[INDEX_FR] = new ModuleIDs(1, 2, 11, -0.3125, false);
            MODULE_CONFIGS[INDEX_BR] = new ModuleIDs(3, 4, 12, 0.0896, false);
            MODULE_CONFIGS[INDEX_BL] = new ModuleIDs(5, 6, 13, 0.4382 + 0.5, false);
            MODULE_CONFIGS[INDEX_FL] = new ModuleIDs(7, 8, 14, -0.1447 + 0.5, false);
        break;

        case SN2:
            MODULE_CONFIGS[INDEX_FR] = new ModuleIDs(1, 2, 11, 0.834229, false); //this one is changing?  0.811035 shifted by 7 degrees?
            MODULE_CONFIGS[INDEX_BR] = new ModuleIDs(3, 4, 12, 0.401855, false);
            MODULE_CONFIGS[INDEX_BL] = new ModuleIDs(5, 6, 13, -0.273926, false);
            MODULE_CONFIGS[INDEX_FL] = new ModuleIDs(7, 8, 14, 0.3394, false);
        break;

        case SN_TEST:
            MODULE_CONFIGS[INDEX_FR] = new ModuleIDs(1, 2, 9, 0.0, false);
            MODULE_CONFIGS[INDEX_BR] = new ModuleIDs(3, 4, 8, 0.0, false);
            MODULE_CONFIGS[INDEX_BL] = new ModuleIDs(5, 6, 7, 0.0, false);
            MODULE_CONFIGS[INDEX_FL] = new ModuleIDs(7, 8, 6, 0.0, false);
        break;

        case BUBBLES:
            MODULE_CONFIGS[INDEX_FR] = new ModuleIDs(1, 2, 11, 9.115966796875 - 9.0, false);//-0.539306640625, false);
            MODULE_CONFIGS[INDEX_BR] = new ModuleIDs(3, 4, 12, 8.167724609375 - 8.0, false);//0.083251953125, false);
            MODULE_CONFIGS[INDEX_BL] = new ModuleIDs(5, 6, 13, 9.37939453125 - 9.0, false);//0.85107421875, false);
            MODULE_CONFIGS[INDEX_FL] = new ModuleIDs(7, 8, 14, 8.743896484375 - 9.0, false);//-0.05224609375, false);
        break;
    }
  }

  // -----------------------------------------------------------------------------------------------------------------------------
  //
  //      Drivebase controller/object definements
  //
  // -----------------------------------------------------------------------------------------------------------------------------


  public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[4];
  static {
    MODULE_TRANSLATIONS[INDEX_FR] = new Translation2d( DRIVE_CONFIG.robotLengthX(), -DRIVE_CONFIG.robotWidthY()).div(2.0);
    MODULE_TRANSLATIONS[INDEX_BR] = new Translation2d(-DRIVE_CONFIG.robotLengthX(), -DRIVE_CONFIG.robotWidthY()).div(2.0);
    MODULE_TRANSLATIONS[INDEX_BL] = new Translation2d(-DRIVE_CONFIG.robotLengthX(),  DRIVE_CONFIG.robotWidthY()).div(2.0);
    MODULE_TRANSLATIONS[INDEX_FL] = new Translation2d( DRIVE_CONFIG.robotLengthX(),  DRIVE_CONFIG.robotWidthY()).div(2.0);
  }

  // calculates the orientation and speed of individual swerve modules when given
  // the motion of the whole robot
  public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

  // -----------------------------------------------------------------------------------------------------------------------------
  //
  //      Trajectory Helpers
  //
  // -----------------------------------------------------------------------------------------------------------------------------
  public static HolonomicDriveController getNewHolController() {
    return new HolonomicDriveController(
      new PIDController(10.0, 0.0, 0.3),
      new PIDController(10.0, 0.0, 0.3),
      new ProfiledPIDController(
        9.0,
        0.0,
        0.3,
        new TrapezoidProfile.Constraints(TRAJECTORY_CONFIG.maxAngularVelocity, TRAJECTORY_CONFIG.maxAngularAcceleration)
      )
    );
  }



  private static final double CARPET_COEF_FRICTION = 800000.0;
  private static final double DRIVE_CURRENT_LIMIT = 400.0;
  public static final double DRIVE_VELOCITY_DEADBAND = 1e-9;
  public static final ChassisSpeeds NON_ZERO_CHASSIS_SPEED = new ChassisSpeeds(1, 1, 0); //TODO should this be smaller?

  public static final double ROBOT_MASS = 60.0;
  public static final double ROBOT_MOI = (2.0 / 12.0) * ROBOT_MASS * (Math.pow(DRIVE_CONFIG.bumperWidthX(), 2));



  // -----------------------------------------------------------------------------------------------------------------------------
  //
  //      Simulation helpers
  //
  // -----------------------------------------------------------------------------------------------------------------------------

  /****************************************************************************************
   *
   * Record and Enum types
   *
   *******************************************************************************************/
  public record ModuleIDs(
      int driveID,
      int steerID,
      int absoluteEncoderChannel,
      double absoluteEncoderOffset,
      boolean encoderInverted) {}

  public record ModuleGainsAndRatios(
      double driveFFkS,
      double driveFFkV,
      double driveFFkT,
      double drivekP,
      double drivekD,
      double steerkP,
      double steerkD,
      double driveReduction,
      double steerReduction) {}

  @Builder
  public record DriveConfig(
      double wheelRadius,
      double robotLengthX,
      double robotWidthY,
      double bumperWidthX,
      double bumperWidthY,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    public double driveBaseRadius() {
      return Math.hypot(robotLengthX / 2.0, robotWidthY / 2.0);
    }
  }

  public enum Mk4iReductions {
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L2_16t(
        (50.0 / 16.0)
            * (17.0 / 27.0)
            * (45.0 / 15.0)), // SDS mk4i L2 ratio reduction plus 16 tooth pinion
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),

    L2_PLUS(6.75 * (14.0 / 16.0)),

    steer((150.0 / 7.0));

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
