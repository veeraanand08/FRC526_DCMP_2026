package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.intake.Intake.PivotState.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import java.util.EnumMap;

public final class IntakeConstants {
  public static final double PIVOT_GEAR_RATIO = 75;
  public static final double PIVOT_KP = 60.0;
  public static final double PIVOT_KI = 0;
  public static final double PIVOT_KD = 0;
  public static final double PIVOT_KS = 0.05;
  public static final double PIVOT_KV = 0.12;
  public static final double PIVOT_KA = 0;
  public static final double PIVOT_KG = 0.0;
  public static final double PIVOT_CRUISE_VELOCITY = 6;
  public static final double PIVOT_CRUISE_ACCELERATION = 10;

  public static final double ROLLER_MOI = 0.002;
  public static final double ROLLER_GEAR_RATIO = 1.0;
  public static final double ROLLER_KP = 0.1;
  public static final double ROLLER_KI = 0;
  public static final double ROLLER_KD = 0;
  public static final double ROLLER_KS = 0;
  public static final double ROLLER_KV = 0.12;

  public static final TalonFXConfiguration PIVOT_CONFIG =
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
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(PIVOT_GEAR_RATIO))
          .withSlot0(
              new Slot0Configs()
                  .withKP(PIVOT_KP)
                  .withKI(PIVOT_KI)
                  .withKD(PIVOT_KD)
                  .withKS(PIVOT_KS)
                  .withKV(PIVOT_KV)
                  .withKG(PIVOT_KG)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(PIVOT_CRUISE_VELOCITY)
                  .withMotionMagicAcceleration(PIVOT_CRUISE_ACCELERATION));

  public static final TalonFXConfiguration ROLLER_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(80)
                  .withSupplyCurrentLimit(40)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withSlot0(
              new Slot0Configs()
                  .withKP(ROLLER_KP)
                  .withKI(ROLLER_KI)
                  .withKD(ROLLER_KD)
                  .withKS(ROLLER_KS)
                  .withKV(ROLLER_KV));

  // setpoints, in degrees
  public static final EnumMap<Intake.PivotState, Angle> SETPOINTS =
      new EnumMap<>(Intake.PivotState.class);

  static {
    SETPOINTS.put(RAISING, Degrees.of(0.0));
    SETPOINTS.put(LOWERING, Degrees.of(137.0));
    SETPOINTS.put(AGITATING, Degrees.of(45.0));
  }

  public static final double AGITATION_UPPER_ANGLE = 10;
  public static final double AGITATION_LOWER_ANGLE = 100;
  public static final double AGITATION_PERIOD = 2; // seconds

  public static final double AGITATION_RAD_PER_SEC = 2.0 * Math.PI / AGITATION_PERIOD;
  public static final double AGITATION_HALF_AMPLITUDE =
      (AGITATION_LOWER_ANGLE - AGITATION_UPPER_ANGLE) / 2.0;
  public static final double AGITATION_MID_ANGLE =
      (AGITATION_LOWER_ANGLE + AGITATION_UPPER_ANGLE) / 2.0;

  public static final double ROLLER_RPS = 5000 / 60.0;
  public static final double ROLLER_RPS_REVERSED = -4500 / 60.0;
  public static final double ROLLER_RPS_SLOW = 4000 / 60.0;

  // physical max
  public static final Angle MAX_ANGLE = Degrees.of(137);

  // sim
  public static final int INTAKE_CAPACITY = 50;
}
