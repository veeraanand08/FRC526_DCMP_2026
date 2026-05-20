package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class ShooterConstants {
  public static final double SHOOTER_MOI = 0.001;
  public static final double SHOOTER_GEAR_RATIO = 1.0;

  public static final double SHOOTER_KP = 0.3;
  public static final double SHOOTER_KI = 0;
  public static final double SHOOTER_KD = 0;
  public static final double SHOOTER_KS = 0.19;
  public static final double SHOOTER_KV = 0.12;

  public static final TalonFXConfiguration SHOOTER_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(80)
                  .withSupplyCurrentLimit(40)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withSlot0(
              new Slot0Configs()
                  .withKP(SHOOTER_KP)
                  .withKI(SHOOTER_KI)
                  .withKD(SHOOTER_KD)
                  .withKS(SHOOTER_KS)
                  .withKV(SHOOTER_KV));

  public static final MotorAlignmentValue BOTTOM_LEFT_ALIGNMENT_VALUE = MotorAlignmentValue.Aligned;
  public static final MotorAlignmentValue TOP_RIGHT_ALIGNMENT_VALUE = MotorAlignmentValue.Opposed;
  public static final MotorAlignmentValue BOTTOM_RIGHT_ALIGNMENT_VALUE =
      MotorAlignmentValue.Opposed;

  public static final double NEGATIVE_RATE_LIMIT = 2000;

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_RPS = new InterpolatingDoubleTreeMap();

  static {
    // Distance, RPS
    DISTANCE_TO_RPS.put(0.0, 3500.0);
  }

  public static final LoggedNetworkNumber DEFAULT_RPM =
      new LoggedNetworkNumber("/Tuning/Shooter/RPM", 3500);

  public static final double AGITATION_TIME = 2.5;

  // sim
  public static final double BPS = 15.0;
}
