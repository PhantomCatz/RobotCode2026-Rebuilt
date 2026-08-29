package frc.robot;
import org.wpilib.math.geometry.*;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Distance;

/**
 * Contains various field dimensions and useful reference points. All units are
 * in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {

  public static final double fieldTrenchX = 4.645359992980957;

  public static final double aprilTagWidth = Units.inchesToMeters(6.5);

  private static final Translation2d HUB_LOCATION = new Translation2d(4.6256194, 4.0346376);
  private static final Translation2d TRENCH_SHOOTING_LOCATION = new Translation2d(4.3802995681762695,
      0.6432812809944153);

  private static final Translation2d RIGHT_CORNER_HOARD = new Translation2d(0.8200110197067261, 0.0);

  // ******CLIMB BLUE ALLIANCE RELATIVE********/
  public static final double TOWER_Y_CENTER = Units.inchesToMeters(146.86);
  private static final double RUNG_SPACING = Units.inchesToMeters(32.250 + 2 * 1.50); // distance from center of climb
                                                                                      // to rung
  private static final double CLIMB_DISTANCE_AWAY = 0.3; // meters, distance from the tower we want to be when climbing

  public static final double BLUE_CLIMB_X_OFFSET = Units.inchesToMeters(2.3);
  public static final double RED_CLIMB_X_OFFSET = -Units.inchesToMeters(2.13); // 1.5 at home 0.0 for Da Vinci

  private static final double LEFT_X_OFFSET = Units.inchesToMeters(0.0); // 1.5 at home
  private static final double RIGHT_X_OFFSET = Units.inchesToMeters(0.0);

  private static final double LEFT_Y_OFFSET = Units.inchesToMeters(0.0); // inwards towards the tower
  private static final double RIGHT_Y_OFFSET = Units.inchesToMeters(0.0);

  public static final Translation2d ROBOT_CLIMB_OFFSET = new Translation2d(Units.inchesToMeters(-3.75),
      Units.inchesToMeters(35.0 / 2 - 4.0));

  private static final double CLIMB_CLOSE_BASE_X = Units.inchesToMeters(40.0 + 3.51 / 2.0); //41.755

  private static final Translation2d CLIMB_CLOSE_RIGHT_TARGET = new Translation2d(
      CLIMB_CLOSE_BASE_X + RIGHT_X_OFFSET,
      TOWER_Y_CENTER - (RUNG_SPACING / 2.0) + RIGHT_Y_OFFSET);

  private static final Translation2d CLIMB_CLOSE_LEFT_TARGET = new Translation2d(
      CLIMB_CLOSE_BASE_X + LEFT_X_OFFSET,
      TOWER_Y_CENTER + (RUNG_SPACING / 2.0) - LEFT_Y_OFFSET);

  private static final Pose2d CLIMB_CLOSE_RIGHT = new Pose2d(
      new Translation2d(
          CLIMB_CLOSE_RIGHT_TARGET.getX() + ROBOT_CLIMB_OFFSET.getX(),
          CLIMB_CLOSE_RIGHT_TARGET.getY() - ROBOT_CLIMB_OFFSET.getY()),
      Rotation2d.k180deg);

  private static final Pose2d CLIMB_CLOSE_LEFT = new Pose2d(
      new Translation2d(
          CLIMB_CLOSE_LEFT_TARGET.getX() - ROBOT_CLIMB_OFFSET.getX(),
          CLIMB_CLOSE_LEFT_TARGET.getY() + ROBOT_CLIMB_OFFSET.getY()),
      new Rotation2d());

  // TODO these are random numbers
  private static final Pose2d CLIMB_RED_LEFT = new Pose2d(new Translation2d(15.520146369934082, 3.3339905738830566), Rotation2d.k180deg);
  private static final Pose2d CLIMB_RED_RIGHT = new Pose2d(new Translation2d(15.520146369934082, 5.333996295928955), new Rotation2d());
  private static final Pose2d CLIMB_BLUE_LEFT = new Pose2d(new Translation2d(1.0474562644958496, 4.761989593505859), new Rotation2d());
  private static final Pose2d CLIMB_BLUE_RIGHT = new Pose2d(new Translation2d(1.0474562644958496, 2.746171474456787), Rotation2d.k180deg);

  private static final Translation2d CLIMB_APRILTAG_POSE = new Translation2d(0.0, TOWER_Y_CENTER);
  /*********************/

  private static final Translation2d RIGHT_CORNER = new Translation2d(0.5085551738739014, 0.5085861086845398);
  private static final double NET_LENGTH = 2.0; // meters
  public static final double NET_LENGTH_HALF = NET_LENGTH / 2.0;

  public static final Distance HUB_HEIGHT = org.wpilib.units.Units.Inches.of(72.0);
  public static final Distance HUB_RIM_RADIUS = org.wpilib.units.Units.Inches.of(41.0 / 2.0);

  public static final double BOTTOM_TRENCH_MAX_Y = 1.143760085105896;
  public static final double LEFT_TRENCH_X = 4.6245880126953125;
  public static final double MIN_RUMBLE_DIST = LEFT_TRENCH_X - 2.846224308013916;


  public static Translation2d getBlueAllianceClimbApriltagLocation(){
    return CLIMB_APRILTAG_POSE;
  }

  // public static Pose2d getClimbClosePosition(Translation2d robotPose) {
  //   Pose2d flippedRight = AllianceFlipUtil.apply(CLIMB_CLOSE_RIGHT);
  //   Pose2d flippedLeft = AllianceFlipUtil.apply(CLIMB_CLOSE_LEFT);

  //   double distRight = robotPose.getDistance(flippedRight.getTranslation());
  //   double distLeft = robotPose.getDistance(flippedLeft.getTranslation());

  //   Pose2d closerPose = (distLeft < distRight) ? flippedLeft : flippedRight;

  //   double xVariationOffset = AllianceFlipUtil.shouldFlip() ? RED_CLIMB_X_OFFSET : BLUE_CLIMB_X_OFFSET;
  //   Translation2d variationTranslation = new Translation2d(xVariationOffset, 0.0);

  //   return new Pose2d(closerPose.getTranslation().plus(variationTranslation), closerPose.getRotation());
  // }


  

  // Depot swipe, needs 1st pos coords
  private static final Pose2d TowerSwipe_Depot_Corner = new Pose2d(
      new Translation2d(
          1.333,
          7.162),
      Rotation2d.k180deg);
  private static final Pose2d TowerSwipe_Depot_Middle = new Pose2d(
      new Translation2d(
          0.641,
          4.835),
      Rotation2d.k180deg);
  // Tower swipe, needs 1st pos coords
  private static final Pose2d TowerSwipe_Outpost = new Pose2d(
      new Translation2d(
          1.12416,
          2.09345),
      Rotation2d.k180deg);




  public static Translation2d getCornerHoardLocation() {
    return RIGHT_CORNER_HOARD;
  }

  }

