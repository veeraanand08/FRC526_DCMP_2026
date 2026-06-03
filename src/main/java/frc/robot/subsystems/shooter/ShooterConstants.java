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
  public static final double SIM_BPS = 15.0;
  public static final double SHOOTER_MOI = 0.01;
  public static final double SHOOTER_GEAR_RATIO = 1.0;
  public static final double EXIT_HEIGHT = Units.inchesToMeters(19.484825); // meters
  public static final double LAUNCH_ANGLE = 65; // degrees
  public static final double WHEEL_DIAMETER = Units.inchesToMeters(4); // meters
  public static final double SLIP_FACTOR = 0.6;
  public static final Translation2d ROBOT_TO_SHOOTER =
      new Translation2d(Units.inchesToMeters(8.397388), 0.0);
  // verify these through testing
  public static final double MIN_DISTANCE = 2.0;
  public static final double MAX_DISTANCE = 6.0;

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
    SHOT_CALC.loadLUTEntry(2.0, 1978.515625, 0.7790177108922681);
    SHOT_CALC.loadLUTEntry(2.05, 2005.859375, 0.7883857291589772);
    SHOT_CALC.loadLUTEntry(2.1, 2019.53125, 0.8028359581895439);
    SHOT_CALC.loadLUTEntry(2.15, 2033.203125, 0.8171265257923973);
    SHOT_CALC.loadLUTEntry(2.2, 2046.875, 0.8312608488158638);
    SHOT_CALC.loadLUTEntry(2.25, 2060.546875, 0.8452422373968368);
    SHOT_CALC.loadLUTEntry(2.3, 2074.21875, 0.8590739158374022);
    SHOT_CALC.loadLUTEntry(2.35, 2087.890625, 0.8727590340861652);
    SHOT_CALC.loadLUTEntry(2.4, 2115.234375, 0.8806847189891566);
    SHOT_CALC.loadLUTEntry(2.45, 2128.90625, 0.8940373218675625);
    SHOT_CALC.loadLUTEntry(2.5, 2142.578125, 0.9072536506357958);
    SHOT_CALC.loadLUTEntry(2.55, 2156.25, 0.9203364447979127);
    SHOT_CALC.loadLUTEntry(2.6, 2169.921875, 0.9332883937124921);
    SHOT_CALC.loadLUTEntry(2.65, 2190.4296875, 0.943218357107363);
    SHOT_CALC.loadLUTEntry(2.7, 2204.1015625, 0.9558955335167068);
    SHOT_CALC.loadLUTEntry(2.75, 2224.609375, 0.9655340789640291);
    SHOT_CALC.loadLUTEntry(2.8, 2238.28125, 0.9779487661997502);
    SHOT_CALC.loadLUTEntry(2.85, 2251.953125, 0.9902461353511629);
    SHOT_CALC.loadLUTEntry(2.9, 2265.625, 1.0024284224309001);
    SHOT_CALC.loadLUTEntry(2.95, 2286.1328125, 1.0115253065068204);
    SHOT_CALC.loadLUTEntry(3.0, 2299.8046875, 1.023466482492644);
    SHOT_CALC.loadLUTEntry(3.05, 2320.3125, 1.0323111961902853);
    SHOT_CALC.loadLUTEntry(3.1, 2333.984375, 1.0440215727839337);
    SHOT_CALC.loadLUTEntry(3.15, 2347.65625, 1.0556282744880898);
    SHOT_CALC.loadLUTEntry(3.2, 2361.328125, 1.0671331728568942);
    SHOT_CALC.loadLUTEntry(3.25, 2381.8359375, 1.0755051945832765);
    SHOT_CALC.loadLUTEntry(3.3, 2395.5078125, 1.0867973589234445);
    SHOT_CALC.loadLUTEntry(3.35, 2416.015625, 1.0949500956498912);
    SHOT_CALC.loadLUTEntry(3.4, 2429.6875, 1.1060382688156545);
    SHOT_CALC.loadLUTEntry(3.45, 2443.359375, 1.1170343266889118);
    SHOT_CALC.loadLUTEntry(3.5, 2457.03125, 1.1279399058683);
    SHOT_CALC.loadLUTEntry(3.55, 2477.5390625, 1.1356781120296227);
    SHOT_CALC.loadLUTEntry(3.6, 2491.2109375, 1.146395064771634);
    SHOT_CALC.loadLUTEntry(3.65, 2511.71875, 1.1539417599232493);
    SHOT_CALC.loadLUTEntry(3.7, 2525.390625, 1.1644775559674592);
    SHOT_CALC.loadLUTEntry(3.75, 2539.0625, 1.1749311245012788);
    SHOT_CALC.loadLUTEntry(3.8, 2552.734375, 1.1853039198724427);
    SHOT_CALC.loadLUTEntry(3.85, 2566.40625, 1.195597288877047);
    SHOT_CALC.loadLUTEntry(3.9, 2586.9140625, 1.2026902910201445);
    SHOT_CALC.loadLUTEntry(3.95, 2600.5859375, 1.2128186178687415);
    SHOT_CALC.loadLUTEntry(4.0, 2614.2578125, 1.2228717612455626);
    SHOT_CALC.loadLUTEntry(4.05, 2634.765625, 1.2297167761095458);
    SHOT_CALC.loadLUTEntry(4.1, 2648.4375, 1.239613899404438);
    SHOT_CALC.loadLUTEntry(4.15, 2662.109375, 1.2494397761137481);
    SHOT_CALC.loadLUTEntry(4.2, 2675.78125, 1.259195569740176);
    SHOT_CALC.loadLUTEntry(4.25, 2696.2890625, 1.2657298923119153);
    SHOT_CALC.loadLUTEntry(4.3, 2709.9609375, 1.2753405559514226);
    SHOT_CALC.loadLUTEntry(4.35, 2723.6328125, 1.2848847288570224);
    SHOT_CALC.loadLUTEntry(4.4, 2737.3046875, 1.294363509706622);
    SHOT_CALC.loadLUTEntry(4.45, 2757.8125, 1.3006106350970708);
    SHOT_CALC.loadLUTEntry(4.5, 2771.484375, 1.309954185333138);
    SHOT_CALC.loadLUTEntry(4.55, 2785.15625, 1.3192356398789173);
    SHOT_CALC.loadLUTEntry(4.6, 2798.828125, 1.3284559493931514);
    SHOT_CALC.loadLUTEntry(4.65, 2819.3359375, 1.3344372594029008);
    SHOT_CALC.loadLUTEntry(4.7, 2833.0078125, 1.343531438963371);
    SHOT_CALC.loadLUTEntry(4.75, 2846.6796875, 1.3525674949907256);
    SHOT_CALC.loadLUTEntry(4.8, 2860.3515625, 1.3615463294921424);
    SHOT_CALC.loadLUTEntry(4.85, 2874.0234375, 1.3704688255318886);
    SHOT_CALC.loadLUTEntry(4.9, 2894.53125, 1.3761423445168899);
    SHOT_CALC.loadLUTEntry(4.95, 2908.203125, 1.3849489205892953);
    SHOT_CALC.loadLUTEntry(5.0, 2921.875, 1.3937018957522977);
    SHOT_CALC.loadLUTEntry(5.05, 2935.546875, 1.4024020551496834);
    SHOT_CALC.loadLUTEntry(5.1, 2949.21875, 1.4110501897800884);
    SHOT_CALC.loadLUTEntry(5.15, 2969.7265625, 1.416442930052865);
    SHOT_CALC.loadLUTEntry(5.2, 2983.3984375, 1.4249843453873163);
    SHOT_CALC.loadLUTEntry(5.25, 2997.0703125, 1.4334762500147942);
    SHOT_CALC.loadLUTEntry(5.3, 3010.7421875, 1.4419193096420089);
    SHOT_CALC.loadLUTEntry(5.35, 3024.4140625, 1.4503143091936086);
    SHOT_CALC.loadLUTEntry(5.4, 3038.0859375, 1.4586619225373072);
    SHOT_CALC.loadLUTEntry(5.45, 3058.59375, 1.4637470371326395);
    SHOT_CALC.loadLUTEntry(5.5, 3072.265625, 1.4719977436317229);
    SHOT_CALC.loadLUTEntry(5.55, 3085.9375, 1.4802032917719548);
    SHOT_CALC.loadLUTEntry(5.6, 3099.609375, 1.4883643032967244);
    SHOT_CALC.loadLUTEntry(5.65, 3113.28125, 1.496481428765764);
    SHOT_CALC.loadLUTEntry(5.7, 3126.953125, 1.504555306806708);
    SHOT_CALC.loadLUTEntry(5.75, 3147.4609375, 1.509363447202504);
    SHOT_CALC.loadLUTEntry(5.8, 3161.1328125, 1.5173491765745235);
    SHOT_CALC.loadLUTEntry(5.85, 3174.8046875, 1.5252936128437697);
    SHOT_CALC.loadLUTEntry(5.9, 3188.4765625, 1.5331973453868801);
    SHOT_CALC.loadLUTEntry(5.95, 3202.1484375, 1.5410609526262717);
    SHOT_CALC.loadLUTEntry(6.0, 3215.8203125, 1.5488850230718576);
  }
}
