package frc.robot;


import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.Utilities.AllianceFlipUtil;
import lombok.Getter;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static AprilTagFieldLayout APRILTAG_LAYOUT = AprilTagLayoutType.OFFICIAL.getLayout();

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
              "Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
        }
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
  }
}
