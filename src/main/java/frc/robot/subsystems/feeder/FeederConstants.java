package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class FeederConstants {
  public static final double INDEXER_MOI = 0.001;
  public static final double KICKER_MOI = 0.001;

  public static final double INDEXER_KP = 0.1;
  public static final double INDEXER_KI = 0;
  public static final double INDEXER_KD = 0;
  public static final double INDEXER_KS = 0;
  public static final double INDEXER_KV = 0.12;

  public static final double KICKER_KP = 0.6;
  public static final double KICKER_KI = 0;
  public static final double KICKER_KD = 0;
  public static final double KICKER_KS = 0;
  public static final double KICKER_KV = 0.12;

  public static final TalonFXConfiguration INDEXER_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimit(40)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withSlot0(
              new Slot0Configs()
                  .withKP(INDEXER_KP)
                  .withKI(INDEXER_KI)
                  .withKD(INDEXER_KD)
                  .withKS(INDEXER_KS)
                  .withKV(INDEXER_KV));

  public static final TalonFXConfiguration KICKER_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(100)
                  .withSupplyCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withSlot0(
              new Slot0Configs()
                  .withKP(KICKER_KP)
                  .withKI(KICKER_KI)
                  .withKD(KICKER_KD)
                  .withKS(KICKER_KS)
                  .withKV(KICKER_KV));

  public static final double INDEXER_RPS = 3500 / 60.0;
  public static final double KICKER_RPS = 5000 / 60.0;
}
