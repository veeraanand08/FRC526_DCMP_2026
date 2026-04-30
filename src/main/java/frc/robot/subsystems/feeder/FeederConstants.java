package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.signals.InvertedValue;

public final class FeederConstants {
  public static final int INDEXER_STATOR_LIMIT = 60;
  public static final int INDEXER_SUPPLY_LIMIT = 40;
  public static final int KICKER_STATOR_LIMIT = 100;
  public static final int KICKER_SUPPLY_LIMIT = 60;

  public static final InvertedValue INDEXER_INVERTED = InvertedValue.Clockwise_Positive;
  public static final InvertedValue KICKER_INVERTED = InvertedValue.Clockwise_Positive;

  public static final double INDEXER_MOI = 0.001;
  public static final double KICKER_MOI = 0.001;
  public static final double KICKER_GEAR_RATIO = 1.0; // adjust

  public static final double INDEXER_KP = 0.1;
  public static final double INDEXER_KI = 0;
  public static final double INDEXER_KD = 0;
  public static final double INDEXER_KS = 0;
  public static final double INDEXER_KV = 0.12;

  public static final double KICKER_KP = 0.1;
  public static final double KICKER_KI = 0;
  public static final double KICKER_KD = 0;
  public static final double KICKER_KS = 0;
  public static final double KICKER_KV = 0.12;

  public static final double INDEXER_RPS = 3500 / 60.0;
  public static final double KICKER_RPS = 5000 / 60.0;
}
