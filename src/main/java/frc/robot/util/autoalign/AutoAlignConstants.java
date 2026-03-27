package frc.robot.util.autoalign;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class AutoAlignConstants {
  public static final double TOLERANCE_DEG = 2.0;

  /* Shoot on the fly */
  public static final int MAX_ITERATIONS = 3;

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_TIME =
      new InterpolatingDoubleTreeMap();

  static {
    // Distance, Time
    DISTANCE_TO_TIME.put(0.0, 0.0);
  }
}
