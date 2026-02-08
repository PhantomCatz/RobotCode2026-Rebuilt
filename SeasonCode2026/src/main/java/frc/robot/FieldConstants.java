package frc.robot;


import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.Utilities.AllianceFlipUtil;
import lombok.Getter;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

  public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  private static final Translation2d HUB_LOCATION = new Translation2d(4.6256194, 4.0346376);

  /**
   * Returns the position of the hub in the correct alliance.
   */
  public static Translation2d getHubLocation(){
    //This apply method correctly accounts for alliance color
    return AllianceFlipUtil.apply(HUB_LOCATION);
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
