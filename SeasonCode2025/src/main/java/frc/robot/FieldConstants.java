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

  public static final double FIELD_LENGTH_MTRS = Units.inchesToMeters(690.876);
  public static final double FIELD_WIDTH       = Units.inchesToMeters(317);
  public static final double STARTING_LINE_X   = Units.inchesToMeters(299.438); // Measured from the inside of starting line

  public static class Processor {
    public static final Pose2d CENTER_FACE        = new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }
  public static class Barge {
    public static final Translation2d FAR_CAGE    = new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d MIDDLE_CAGE = new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d CLOSE_CAGE  = new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double DEEP_HEIGHT    = Units.inchesToMeters(3.125);
    public static final double SHALLOW_HEIGHT = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final Pose2d leftCenterFace =
        new Pose2d(
            1.197,
            7.028,
            Rotation2d.fromDegrees(-53.673));
    public static final Pose2d rightCenterFace =
        new Pose2d(
            1.197,
            0.950,
            Rotation2d.fromDegrees(53.673));


    public static Pose2d getRightStation(){
      if(AllianceFlipUtil.shouldFlipToRed()){
        return AllianceFlipUtil.apply(rightCenterFace);
      }else{
        return rightCenterFace;
      }
    }

    public static Pose2d getLeftStation(){
      if(AllianceFlipUtil.shouldFlipToRed()){
        return AllianceFlipUtil.apply(leftCenterFace);
      }else{
        return leftCenterFace;
      }
    }
  }

  public static class Reef {
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line
    public static final double scoringDistance = Units.inchesToMeters(21); // Scoring distance from the side of the reef
    public static final double backDistance = Units.inchesToMeters(12);
    public static final double reefOrthogonalRadius =
        Units.inchesToMeters(32.305); // Distance from the center to the side of the reef
    public static final double leftRightDistance = Units.inchesToMeters(6.625); // Center of each face to the left and right scoring positions of each face
  }

  public static class StartPoses{

    public static final Translation2d start1 = new Translation2d(7.253, 7.256);
    public static final Translation2d start2 = new Translation2d(7.253, 6.141);
    public static final Translation2d start3 = new Translation2d(7.253, 5.050);
    public static final Translation2d start4 = new Translation2d(7.253, 4.019);
    public static final Translation2d start5 = new Translation2d(7.253, 3.024);
    public static final Translation2d start6 = new Translation2d(7.253, 1.897);
    public static final Translation2d start7 = new Translation2d(7.253, 0.794);

    public static final Translation2d[] startArray = new Translation2d[] {
      start1,
      start2,
      start3,
      start4,
      start5,
      start6,
      start7
    };
  }

  public static class Net{
    private static final double x = 7.768;// meters
    private static final double yLeft = 7.507; //left and right is from the driverstation pov
    private static final double yRight = 4.606;

    public static double getX(){
      if(AllianceFlipUtil.shouldFlipToRed()){
        return FieldConstants.FIELD_LENGTH_MTRS - x;
      }else{
        return x;
      }
    }

    public static double getYLeft(){
      if(AllianceFlipUtil.shouldFlipToRed()){
        return FieldConstants.FIELD_WIDTH - yLeft;
      }else{
        return yLeft;
      }
    }
    public static double getYRight(){
      if(AllianceFlipUtil.shouldFlipToRed()){
        return FieldConstants.FIELD_WIDTH - yRight;
      }else{
        return yRight;
      }
    }

    /**
     * Returns the closest net position assuming no obstacles
     * @param y Y position of the robot
     * @return
     */
    public static Translation2d getGuessClosestNetPose(double y){
      double guessY;
      if(AllianceFlipUtil.shouldFlipToRed()){
        guessY = MathUtil.clamp(y, getYLeft(), getYRight());
      }else{
        guessY = MathUtil.clamp(y, getYRight(), getYLeft());
      }
      return new Translation2d(getX(), guessY);
    }
  }


  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream =   new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d middleIceCream = new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d rightIceCream =  new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

  public enum ReefHeight {
    L4(Units.inchesToMeters(72), -90),
    L3(Units.inchesToMeters(47.625), -35),
    L2(Units.inchesToMeters(31.875), -35),
    L1(Units.inchesToMeters(18), 0);

    ReefHeight(double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // in degrees
    }

    public final double height;
    public final double pitch;
  }

 public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;
  public static final int aprilTagCount = 22;

  @Getter
  public enum AprilTagLayoutType {
    OFFICIAL("2025-official");

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
