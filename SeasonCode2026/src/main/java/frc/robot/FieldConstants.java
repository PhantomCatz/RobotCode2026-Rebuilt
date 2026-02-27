package frc.robot;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.TurretConstants;
import frc.robot.Utilities.AllianceFlipUtil;
import lombok.Getter;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();
  public static final double fieldYHalf = fieldWidth / 2.0;

  public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  private static final Translation2d HUB_LOCATION = new Translation2d(4.6256194, 4.0346376);
  private static final Translation2d TRENCH_SHOOTING_LOCATION = new Translation2d(4.3802995681762695, 0.6432812809944153);

  private static final Translation2d RIGHT_CORNER_HOARD = new Translation2d(0.8200110197067261, 0.698647677898407);

  private static final Pose2d CLIMB_BACK_AWAY_LEFT = new Pose2d(1.1506646871566772, 1.8027223348617554, Rotation2d.k180deg);
  private static final Pose2d CLIMB_CLOSE_LEFT = new Pose2d(1.1707948446273804, 2.8092310428619385, Rotation2d.k180deg);

  private static final Translation2d CLIMB_TURRET_TRACKING_LOCATION = new Translation2d(0, fieldYHalf);
  private static final Translation2d RIGHT_CORNER = new Translation2d(0.5798526406288147, 0.503104567527771);
  private static final double NET_LENGTH = 2.0; //meters
  public static final double NET_LENGTH_HALF = NET_LENGTH / 2.0;
  private static final Translation2d NET_POS = new Translation2d(5.527492523193359, fieldYHalf);

  public static final Distance HUB_HEIGHT = edu.wpi.first.units.Units.Inches.of(72.0);
  public static final Distance HUB_RIM_RADIUS = edu.wpi.first.units.Units.Inches.of(41.0/2.0);
  public static final double HEIGHT_DIFF = FieldConstants.HUB_HEIGHT.in(edu.wpi.first.units.Units.Meters) - TurretConstants.TURRET_HEIGHT.in(edu.wpi.first.units.Units.Meters);

  /**
   * Returns the position of the hub in the correct alliance.
   */
  public static Translation2d getHubLocation(){
    //This apply method correctly accounts for alliance color
    return AllianceFlipUtil.apply(HUB_LOCATION);
  }

  public static Translation2d getTrenchShootingLocation(){
    return AllianceFlipUtil.apply(TRENCH_SHOOTING_LOCATION);
  }

  public static Translation2d getRightCornerHoardLocation(){
    return AllianceFlipUtil.apply(RIGHT_CORNER_HOARD);
  }

  public static Translation2d getNetLocation(){
    return AllianceFlipUtil.apply(NET_POS);
  }

  public static Pose2d getClimbBackAwayPosition(boolean isRight) {
    Pose2d pose = CLIMB_BACK_AWAY_LEFT;
    if (!isRight) {
      pose = new Pose2d(pose.getX(), fieldWidth - pose.getY(), Rotation2d.kZero); //TODO the climb tower is not the middle of the field
    }
    return AllianceFlipUtil.apply(pose);
  }

  public static Pose2d getClimbClosePosition(boolean isRight) {
    Pose2d pose = CLIMB_CLOSE_LEFT;
    if (!isRight) {
      pose = new Pose2d(pose.getX(), fieldWidth - pose.getY(), Rotation2d.kZero);
    }
    return AllianceFlipUtil.apply(pose);
  }

  public static Translation2d getCorner(boolean isRight){
    Translation2d allianceFlipped = AllianceFlipUtil.apply(RIGHT_CORNER);
    if(isRight){
      return allianceFlipped;
    }else{
      return new Translation2d(allianceFlipped.getX(), fieldWidth - allianceFlipped.getY());
    }
  }

  public static Translation2d getClimbTurretTrackingLocation() {
    return AllianceFlipUtil.apply(CLIMB_TURRET_TRACKING_LOCATION);
  }

  @Getter
  public enum AprilTagLayoutType {

    OFFICIAL("2026-official");

    AprilTagLayoutType(String name) {
      if (CatzConstants.disableHAL) {
        layout = null;
      } else {
          layout =  AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
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
