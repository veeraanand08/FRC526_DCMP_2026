package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class ShooterConstants {
  public static final double SHOOTER_MOI = 0.001;
  public static final double SHOOTER_GEAR_RATIO = 1.0;

  public static final int SHOOTER_STATOR_LIMIT = 100;
  public static final int SHOOTER_SUPPLY_LIMIT = 60;
  public static final InvertedValue SHOOTER_LEFT_INVERTED = InvertedValue.CounterClockwise_Positive;
  public static final double SHOOTER_KP = 0.2;
  public static final double SHOOTER_KI = 0;
  public static final double SHOOTER_KD = 0;
  public static final double SHOOTER_KS = 0.19;
  public static final double SHOOTER_KV = 0.12;

  public static final MotorAlignmentValue BOTTOM_LEFT_ALIGNMENT_VALUE = MotorAlignmentValue.Aligned;
  public static final MotorAlignmentValue TOP_RIGHT_ALIGNMENT_VALUE = MotorAlignmentValue.Opposed;
  public static final MotorAlignmentValue BOTTOM_RIGHT_ALIGNMENT_VALUE =
      MotorAlignmentValue.Opposed;

  public static final double NEGATIVE_RATE_LIMIT = 2000;

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_RPS = new InterpolatingDoubleTreeMap();

  static {
    // Distance, RPS
    DISTANCE_TO_RPS.put(0.0, 0.0);
  }

  public static final LoggedNetworkNumber SHOOTER_DEFAULT_RPM =
      new LoggedNetworkNumber("/Tuning/Shooter/RPM", 3500);
}
