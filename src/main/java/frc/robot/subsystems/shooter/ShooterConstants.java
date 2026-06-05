package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.sotm.ProjectileSimulator;
import frc.robot.util.sotm.ShotCalculator;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class ShooterConstants {
  // physical constants
  public static final double SHOOTER_MOI = 0.01;
  public static final double SHOOTER_GEAR_RATIO = 1.0;
  public static final double EXIT_HEIGHT = Units.inchesToMeters(19.484825); // meters
  public static final double LAUNCH_ANGLE = 65; // degrees
  public static final double WHEEL_DIAMETER = Units.inchesToMeters(4); // meters
  public static final double SLIP_FACTOR = 0.515;
  public static final Translation2d ROBOT_TO_SHOOTER =
      new Translation2d(Units.inchesToMeters(8.397388), 0.0);
  // verify these through testing
  public static final double MIN_DISTANCE = 2.0;
  public static final double MAX_DISTANCE = 6.0;

  public static final double SHOOTER_KP = 0.3;
  public static final double SHOOTER_KI = 0;
  public static final double SHOOTER_KD = 0;
  public static final double SHOOTER_KS = 0;
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

  public static final LoggedNetworkNumber DEFAULT_RPM =
      new LoggedNetworkNumber("/Tuning/Shooter/RPM", 2500);

  public static final double TIME_UNTIL_AGITATION = 2.5;

  // shot calculation
  private static final ShotCalculator.Config SHOT_CALC_CONFIG = new ShotCalculator.Config();

  static {
    SHOT_CALC_CONFIG.launcherOffsetX =
        ROBOT_TO_SHOOTER.getX(); // how far forward the launcher is from robot center (m)
    SHOT_CALC_CONFIG.launcherOffsetY = ROBOT_TO_SHOOTER.getY(); // how far left, 0 if centered
    SHOT_CALC_CONFIG.minScoringDistance = MIN_DISTANCE;
    SHOT_CALC_CONFIG.maxScoringDistance = MAX_DISTANCE;
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
    loadShooterData();
  }

  public static final double SIM_BPS = 15.0;

  public static void generateLookupTable() {
    System.out.println("\nGenerating lookup table...");

    ProjectileSimulator.SimParameters params =
        new ProjectileSimulator.SimParameters(
            0.215, // ball mass kg
            0.1501, // ball diameter m
            0.47, // drag coeff (smooth sphere)
            0.2, // Magnus coeff
            1.225, // air density
            EXIT_HEIGHT, // exit height (m), floor to where the ball leaves the shooter
            WHEEL_DIAMETER, // flywheel diameter (m), measure with calipers
            1.83, // target height (m), from game manual
            SLIP_FACTOR, // slip factor (0=no grip, 1=perfect), tune this on the real robot
            LAUNCH_ANGLE, // launch angle from horizontal, measure from CAD
            0.001, // sim timestep
            1500, // min RPM
            5000, // max RPM
            25, // iterations
            5.0 // max sim time
            );

    ProjectileSimulator sim = new ProjectileSimulator(params, -1.0);
    long startNs = System.nanoTime();
    ProjectileSimulator.GeneratedLUT lut = sim.generateLUT(MIN_DISTANCE, MAX_DISTANCE, 0.05);
    long elapsedMs = (System.nanoTime() - startNs) / 1_000_000;

    // print data
    System.out.println("Generated lookup table:");
    for (var entry : lut.entries()) {
      if (entry.reachable()) {
        System.out.println(
            "SHOT_CALC.loadLUTEntry("
                + entry.distanceM()
                + ", "
                + entry.rpm()
                + ", "
                + entry.tof()
                + ");");
      }
    }
    System.out.println("End of data");
    System.out.println("Finished in " + elapsedMs + " ms");
    System.out.println(
        "\nShutting down, remove call to ShooterConstants.generateLookupTable() to run robot.");
    System.exit(0);
  }

  public static void loadShooterData() {
    // code generated from projectile sim
    SHOT_CALC.loadLUTEntry(2.0, 2320.3125, 0.7739956102989046);
    SHOT_CALC.loadLUTEntry(2.05, 2333.984375, 0.7893594874944103);
    SHOT_CALC.loadLUTEntry(2.1, 2347.65625, 0.804578523025466);
    SHOT_CALC.loadLUTEntry(2.15, 2361.328125, 0.8196555262915997);
    SHOT_CALC.loadLUTEntry(2.2, 2388.671875, 0.8299085287568623);
    SHOT_CALC.loadLUTEntry(2.25, 2402.34375, 0.8446534868413981);
    SHOT_CALC.loadLUTEntry(2.3, 2416.015625, 0.8592656453988821);
    SHOT_CALC.loadLUTEntry(2.35, 2443.359375, 0.8689540839770549);
    SHOT_CALC.loadLUTEntry(2.4, 2457.03125, 0.8832560309930083);
    SHOT_CALC.loadLUTEntry(2.45, 2470.703125, 0.8974336159631003);
    SHOT_CALC.loadLUTEntry(2.5, 2498.046875, 0.9065989863205691);
    SHOT_CALC.loadLUTEntry(2.55, 2511.71875, 0.9204862590926954);
    SHOT_CALC.loadLUTEntry(2.6, 2532.2265625, 0.9317849352464215);
    SHOT_CALC.loadLUTEntry(2.65, 2552.734375, 0.9429368730864898);
    SHOT_CALC.loadLUTEntry(2.7, 2566.40625, 0.9564353992163118);
    SHOT_CALC.loadLUTEntry(2.75, 2593.75, 0.9648148997414732);
    SHOT_CALC.loadLUTEntry(2.8, 2607.421875, 0.9780532883998415);
    SHOT_CALC.loadLUTEntry(2.85, 2621.09375, 0.9911867978859886);
    SHOT_CALC.loadLUTEntry(2.9, 2648.4375, 0.999137420108219);
    SHOT_CALC.loadLUTEntry(2.95, 2662.109375, 1.012026476908376);
    SHOT_CALC.loadLUTEntry(3.0, 2682.6171875, 1.0222580179028034);
    SHOT_CALC.loadLUTEntry(3.05, 2703.125, 1.0323679435952209);
    SHOT_CALC.loadLUTEntry(3.1, 2716.796875, 1.0449282052967364);
    SHOT_CALC.loadLUTEntry(3.15, 2737.3046875, 1.054807961360004);
    SHOT_CALC.loadLUTEntry(3.2, 2757.8125, 1.0645740134650843);
    SHOT_CALC.loadLUTEntry(3.25, 2771.484375, 1.076824418745364);
    SHOT_CALC.loadLUTEntry(3.3, 2791.9921875, 1.0863743328305766);
    SHOT_CALC.loadLUTEntry(3.35, 2812.5, 1.095817801373177);
    SHOT_CALC.loadLUTEntry(3.4, 2826.171875, 1.107775835301225);
    SHOT_CALC.loadLUTEntry(3.45, 2846.6796875, 1.1170161013204163);
    SHOT_CALC.loadLUTEntry(3.5, 2867.1875, 1.1261566463718071);
    SHOT_CALC.loadLUTEntry(3.55, 2880.859375, 1.1378384736450786);
    SHOT_CALC.loadLUTEntry(3.6, 2901.3671875, 1.1467877691367823);
    SHOT_CALC.loadLUTEntry(3.65, 2921.875, 1.1556435094998043);
    SHOT_CALC.loadLUTEntry(3.7, 2935.546875, 1.1670641108404547);
    SHOT_CALC.loadLUTEntry(3.75, 2956.0546875, 1.1757396725548028);
    SHOT_CALC.loadLUTEntry(3.8, 2976.5625, 1.1843273841442756);
    SHOT_CALC.loadLUTEntry(3.85, 2990.234375, 1.1955007023052298);
    SHOT_CALC.loadLUTEntry(3.9, 3010.7421875, 1.2039184690448435);
    SHOT_CALC.loadLUTEntry(3.95, 3031.25, 1.2122537025881825);
    SHOT_CALC.loadLUTEntry(4.0, 3044.921875, 1.2231926644958533);
    SHOT_CALC.loadLUTEntry(4.05, 3065.4296875, 1.2313674819105824);
    SHOT_CALC.loadLUTEntry(4.1, 3085.9375, 1.2394646480183202);
    SHOT_CALC.loadLUTEntry(4.15, 3099.609375, 1.2501813109209257);
    SHOT_CALC.loadLUTEntry(4.2, 3120.1171875, 1.2581269080521802);
    SHOT_CALC.loadLUTEntry(4.25, 3140.625, 1.2659994082494543);
    SHOT_CALC.loadLUTEntry(4.3, 3154.296875, 1.2765050505468296);
    SHOT_CALC.loadLUTEntry(4.35, 3174.8046875, 1.2842342269971299);
    SHOT_CALC.loadLUTEntry(4.4, 3195.3125, 1.291894546556854);
    SHOT_CALC.loadLUTEntry(4.45, 3208.984375, 1.3021996389930532);
    SHOT_CALC.loadLUTEntry(4.5, 3229.4921875, 1.3097243221935468);
    SHOT_CALC.loadLUTEntry(4.55, 3243.1640625, 1.3199047014189496);
    SHOT_CALC.loadLUTEntry(4.6, 3263.671875, 1.3272984739448264);
    SHOT_CALC.loadLUTEntry(4.65, 3277.34375, 1.3373581011746238);
    SHOT_CALC.loadLUTEntry(4.7, 3297.8515625, 1.3446254751102167);
    SHOT_CALC.loadLUTEntry(4.75, 3318.359375, 1.3518327074486056);
    SHOT_CALC.loadLUTEntry(4.8, 3332.03125, 1.3617134357230802);
    SHOT_CALC.loadLUTEntry(4.85, 3352.5390625, 1.3688008551230677);
    SHOT_CALC.loadLUTEntry(4.9, 3366.2109375, 1.3785701496873024);
    SHOT_CALC.loadLUTEntry(4.95, 3386.71875, 1.385541817796566);
    SHOT_CALC.loadLUTEntry(5.0, 3400.390625, 1.3952030715875856);
    SHOT_CALC.loadLUTEntry(5.05, 3420.8984375, 1.4020628715516061);
    SHOT_CALC.loadLUTEntry(5.1, 3441.40625, 1.4088700080736543);
    SHOT_CALC.loadLUTEntry(5.15, 3455.078125, 1.4183710825298308);
    SHOT_CALC.loadLUTEntry(5.2, 3475.5859375, 1.4250720003295023);
    SHOT_CALC.loadLUTEntry(5.25, 3489.2578125, 1.434473147851532);
    SHOT_CALC.loadLUTEntry(5.3, 3509.765625, 1.4410713730221116);
    SHOT_CALC.loadLUTEntry(5.35, 3523.4375, 1.4503755558109963);
    SHOT_CALC.loadLUTEntry(5.4, 3543.9453125, 1.4568744659365984);
    SHOT_CALC.loadLUTEntry(5.45, 3557.6171875, 1.4660845263253983);
    SHOT_CALC.loadLUTEntry(5.5, 3578.125, 1.4724873817692135);
    SHOT_CALC.loadLUTEntry(5.55, 3591.796875, 1.4816060917749172);
    SHOT_CALC.loadLUTEntry(5.6, 3612.3046875, 1.487915953379567);
    SHOT_CALC.loadLUTEntry(5.65, 3625.9765625, 1.496945962957519);
    SHOT_CALC.loadLUTEntry(5.7, 3646.484375, 1.5031658766416136);
    SHOT_CALC.loadLUTEntry(5.75, 3660.15625, 1.5121097438821205);
    SHOT_CALC.loadLUTEntry(5.8, 3680.6640625, 1.5182425702937057);
    SHOT_CALC.loadLUTEntry(5.85, 3701.171875, 1.5243353827297323);
    SHOT_CALC.loadLUTEntry(5.9, 3714.84375, 1.53315126793249);
    SHOT_CALC.loadLUTEntry(5.95, 3728.515625, 1.5419301655342659);
    SHOT_CALC.loadLUTEntry(6.0, 3749.0234375, 1.5478970315410814);
  }
}
