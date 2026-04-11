package frc.robot;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.TurretConstants;
import frc.robot.Utilities.AllianceFlipUtil;
import lombok.Getter;

/**
 * Contains various field dimensions and useful reference points. All units are
 * in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();
  public static final double fieldXHalf = fieldLength / 2.0;
  public static final double fieldYHalf = fieldWidth / 2.0;
  public static final double fieldTrenchX = 4.645359992980957;

  public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  private static final Translation2d HUB_LOCATION = new Translation2d(4.6256194, 4.0346376);
  private static final Translation2d TRENCH_SHOOTING_LOCATION = new Translation2d(4.3802995681762695,
      0.6432812809944153);

  private static final Translation2d RIGHT_CORNER_HOARD = new Translation2d(0.8200110197067261, 1.698647677898407);

  // ******CLIMB BLUE ALLIANCE RELATIVE********/
  public static final double TOWER_Y_CENTER = Units.inchesToMeters(146.86);
  private static final double RUNG_SPACING = Units.inchesToMeters(32.250 + 2 * 1.50); // distance from center of climb
                                                                                      // to rung
  private static final double CLIMB_DISTANCE_AWAY = 0.3; // meters, distance from the tower we want to be when climbing

  public static final double BLUE_CLIMB_X_OFFSET = Units.inchesToMeters(2.8);
  public static final double RED_CLIMB_X_OFFSET = -Units.inchesToMeters(2.8); // 1.5 at home 0.0 for Da Vinci

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

  private static final Translation2d CLIMB_APRILTAG_POSE = new Translation2d(0.0, TOWER_Y_CENTER);
  /*********************/

  private static final Translation2d RIGHT_CORNER = new Translation2d(0.5085551738739014, 0.5085861086845398);
  private static final double NET_LENGTH = 2.0; // meters
  public static final double NET_LENGTH_HALF = NET_LENGTH / 2.0;
  private static final Translation2d NET_POS = new Translation2d(5.527492523193359, fieldYHalf);

  public static final Distance HUB_HEIGHT = edu.wpi.first.units.Units.Inches.of(72.0);
  public static final Distance HUB_RIM_RADIUS = edu.wpi.first.units.Units.Inches.of(41.0 / 2.0);
  public static final double HEIGHT_DIFF = FieldConstants.HUB_HEIGHT.in(edu.wpi.first.units.Units.Meters)
      - TurretConstants.TURRET_HEIGHT.in(edu.wpi.first.units.Units.Meters);

  public static final double BOTTOM_TRENCH_MAX_Y = 1.143760085105896;
  public static final double TOP_TRENCH_MIN_Y = fieldWidth - BOTTOM_TRENCH_MAX_Y;
  public static final double LEFT_TRENCH_X = 4.6245880126953125;
  public static final double RIGHT_TRENCH_X = fieldLength - LEFT_TRENCH_X;
  public static final double MIN_RUMBLE_DIST = LEFT_TRENCH_X - 2.846224308013916;

  public static Translation2d getClimbApriltagLocation() {
    return AllianceFlipUtil.apply(CLIMB_APRILTAG_POSE);
  }

  public static Translation2d getBlueAllianceClimbApriltagLocation(){
    return CLIMB_APRILTAG_POSE;
  }

  public static Pose2d getClimbClosePosition(Translation2d robotPose) {
    Pose2d flippedRight = AllianceFlipUtil.apply(CLIMB_CLOSE_RIGHT);
    Pose2d flippedLeft = AllianceFlipUtil.apply(CLIMB_CLOSE_LEFT);

    double distRight = robotPose.getDistance(flippedRight.getTranslation());
    double distLeft = robotPose.getDistance(flippedLeft.getTranslation());

    Pose2d closerPose = (distLeft < distRight) ? flippedLeft : flippedRight;

    double xVariationOffset = AllianceFlipUtil.shouldFlip() ? RED_CLIMB_X_OFFSET : BLUE_CLIMB_X_OFFSET;
    Translation2d variationTranslation = new Translation2d(xVariationOffset, 0.0);

    return new Pose2d(closerPose.getTranslation().plus(variationTranslation), closerPose.getRotation());
  }

  public static Pose2d getClimbAwayPosition(Translation2d robotPose) {
    Pose2d closePose = getClimbClosePosition(robotPose);

    double awayY = (closePose.getY() < TOWER_Y_CENTER) ? -CLIMB_DISTANCE_AWAY : CLIMB_DISTANCE_AWAY;
    Translation2d awayTranslation = new Translation2d(0.0, awayY);

    return new Pose2d(closePose.getTranslation().plus(awayTranslation), closePose.getRotation());
  }

  // Depot swipe, needs 1st pos coords
  private static final Pose2d TowerSwipe_Depot_Corner = new Pose2d(
      new Translation2d(
          2.63,
          7.08),
      Rotation2d.k180deg);
  private static final Pose2d TowerSwipe_Depot_Middle = new Pose2d(
      new Translation2d(
          0.836,
          4.778),
      Rotation2d.k180deg);
  // Tower swipe, needs 1st pos coords
  private static final Pose2d TowerSwipe_Outpost = new Pose2d(
      new Translation2d(
          1.12416,
          2.09345),
      Rotation2d.k180deg);

  public static Pose2d getTowerSwipePosition(Translation2d robotPose) {
    Pose2d closePose = getTowerPosition(robotPose);

    double awayY = (closePose.getY() < TOWER_Y_CENTER) ? -CLIMB_DISTANCE_AWAY : CLIMB_DISTANCE_AWAY; // Climb is nice
                                                                                                     // and centered, so
                                                                                                     // uses that
    Translation2d awayTranslation = new Translation2d(0.0, awayY);

    return new Pose2d(closePose.getTranslation().plus(awayTranslation), closePose.getRotation());
  }

  public static Pose2d getTowerPosition(Translation2d robotPose) {
    Pose2d flippedOutpost = AllianceFlipUtil.apply(TowerSwipe_Outpost);

    return new Pose2d(flippedOutpost.getTranslation(), flippedOutpost.getRotation());
  }

  public static int getCloserSwipe(Translation2d robotPose) {
    Pose2d flippedOutpost = AllianceFlipUtil.apply(TowerSwipe_Outpost);
    Pose2d flippedDepot_Middle = AllianceFlipUtil.apply(TowerSwipe_Depot_Middle);
    Pose2d flippedDepot_Corner = AllianceFlipUtil.apply(TowerSwipe_Depot_Corner);

    double distOutpost = robotPose.getDistance(flippedOutpost.getTranslation());
    double distDepot_Middle = robotPose.getDistance(flippedDepot_Middle.getTranslation());
    double distDepot_Corner = robotPose.getDistance(flippedDepot_Corner.getTranslation());
    // System.out.println("Dist outpost: " + distOutpost);
    // System.out.println("Dist depotMid: " + distDepot_Middle);
    // System.out.println("Dist depot: " + distDepot_Corner);
    if (distOutpost < distDepot_Corner) {
      if (distOutpost < distDepot_Middle) {
        System.out.println(1);
        return 1; // outpost
      } else {
        return 2; // middle depot
      }
    } else {
      return 3; // outpost
    }
  }

  public static Translation2d getOpposingHubLocation(){
    if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
      return HUB_LOCATION;
    }else{
      return AllianceFlipUtil.applyNoCondition(HUB_LOCATION);
    }
  }

  /**
   * Returns the position of the hub in the correct alliance.
   */
  public static Translation2d getHubLocation() {
    // This apply method correctly accounts for alliance color
    return AllianceFlipUtil.apply(HUB_LOCATION);
  }

  public static Translation2d getTrenchShootingLocation() {
    return AllianceFlipUtil.apply(TRENCH_SHOOTING_LOCATION);
  }

  public static Translation2d getRightCornerHoardLocation() {
    return AllianceFlipUtil.apply(RIGHT_CORNER_HOARD);
  }

  public static Translation2d getNetLocation() {
    return AllianceFlipUtil.apply(NET_POS);
  }

  public static Translation2d getCorner(boolean isRight) {
    Translation2d allianceFlipped = AllianceFlipUtil.apply(RIGHT_CORNER);
    if (isRight) {
      return allianceFlipped;
    } else {
      return new Translation2d(allianceFlipped.getX(), fieldWidth - allianceFlipped.getY());
    }
  }

  @Getter
  public enum AprilTagLayoutType {

    OFFICIAL("2026-official");

    AprilTagLayoutType(String name) {
      if (CatzConstants.disableHAL) {
        layout = null;
      } else {
        layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
      }
      if (layout == null) {
        layoutString = "";
      } else {
        try {
          layoutString = new ObjectMapper().writeValueAsString(layout);
        } catch (JsonProcessingException e) {
          throw new RuntimeException(
              "Failed to serialize AprilTag layout JSON " + toString() + "for CatzVision");
        }
      }
    }

    public final AprilTagFieldLayout layout;
    private final String layoutString;
  }
}
