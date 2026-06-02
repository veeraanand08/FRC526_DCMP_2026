package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.sotm.ShotCalculator;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class ShooterConstants {
  // physical constants
  public static final double SHOOTER_MOI = 0.001;
  public static final double SHOOTER_GEAR_RATIO = 1.0;
  // verify the following with CAD, not sure if these are correct:
  public static final double EXIT_HEIGHT = .45; // meters
  public static final double LAUNCH_ANGLE = 65; // degrees
  public static final double WHEEL_DIAMETER = Units.inchesToMeters(4); // meters
  public static final double SLIP_FACTOR = 0.6;
  public static final Translation2d ROBOT_TO_SHOOTER =
      new Translation2d(Units.inchesToMeters(12), Units.inchesToMeters(0));

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

  public static final double NEGATIVE_RATE_LIMIT = 2000;

  private static final ShotCalculator.Config SHOT_CALC_CONFIG = new ShotCalculator.Config();

  static {
    SHOT_CALC_CONFIG.launcherOffsetX =
        ROBOT_TO_SHOOTER.getX(); // how far forward the launcher is from robot center (m)
    SHOT_CALC_CONFIG.launcherOffsetY = ROBOT_TO_SHOOTER.getY(); // how far left, 0 if centered
    SHOT_CALC_CONFIG.phaseDelayMs = 30.0; // your vision pipeline latency
    SHOT_CALC_CONFIG.mechLatencyMs = 20.0; // how long the mechanism takes to respond
    SHOT_CALC_CONFIG.maxTiltDeg = 5.0; // suppress firing when chassis tilts past this (bumps/ramps)
    SHOT_CALC_CONFIG.headingSpeedScalar =
        1.0; // heading tolerance tightens with robot speed (0 to disable)
    SHOT_CALC_CONFIG.headingReferenceDistance =
        2.5; // heading tolerance scales with distance from hub
  }

  public static final ShotCalculator SHOT_CALC = new ShotCalculator(SHOT_CALC_CONFIG);

  static {
    // code generated from projectile sim goes here
  }

  public static final LoggedNetworkNumber DEFAULT_RPM =
      new LoggedNetworkNumber("/Tuning/Shooter/RPM", 3500);

  public static final double AGITATION_TIME = 2.5;

  // sim
  public static final double BPS = 15.0;
}
