package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
  public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(
          AprilTagFields
              .k2026RebuiltAndymark); // Chesapeake = AndyMark, FIRST Championship = Welded

  // Camera names, must match names configured on coprocessor
  public static final String CAMERA_0_NAME = "Front";
  public static final String CAMERA_1_NAME = "Side_Left";
  public static final String CAMERA_2_NAME = "Side_Right";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static final Transform3d CAMERA_0_OFFSET =
      new Transform3d(
          Units.inchesToMeters(13.69), // x, forward
          Units.inchesToMeters(0.0), // y, left
          Units.inchesToMeters(13.77), // z, up
          new Rotation3d(
              Units.degreesToRadians(0.0), // roll
              Units.degreesToRadians(-20.0), // pitch
              Units.degreesToRadians(0.0))); // yaw
  public static final Transform3d CAMERA_1_OFFSET =
      new Transform3d(
          Units.inchesToMeters(-12.5), // x, forward
          Units.inchesToMeters(-16), // y, left
          Units.inchesToMeters(7.5), // z, up
          new Rotation3d(
              Units.degreesToRadians(0.0), // roll
              Units.degreesToRadians(-1.6), // pitch
              Units.degreesToRadians(-150.0))); // yaw
  public static final Transform3d CAMERA_2_OFFSET =
      new Transform3d(
          Units.inchesToMeters(-12.5), // x, forward
          Units.inchesToMeters(-16), // y, left
          Units.inchesToMeters(7.5), // z, up
          new Rotation3d(
              Units.degreesToRadians(2.0), // roll
              Units.degreesToRadians(-6.0), // pitch
              Units.degreesToRadians(150.0))); // yaw

  // Basic filtering thresholds
  public static final double MAX_AMBIGUITY = 0.3;
  public static final double MAX_Z_ERROR = 0.75;

  // Standard deviation baselines, for 1-meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
  public static final double ANGULAR_STD_DEV_BASELINE = 0.6; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static final double[] CAMERA_STD_DEV_FACTORS = new double[] {1.0, 1.0, 1.0};

  // Multipliers to apply for MegaTag 2 observations (Limelight only)
  public static final double LINEAR_STD_DEV_MEGATAG_2_FACTOR =
      0.5; // More stable than full 3D solve
  public static final double ANGULAR_STD_DEV_MEGATAG_2_FACTOR =
      Double.POSITIVE_INFINITY; // No rotation data available
}
