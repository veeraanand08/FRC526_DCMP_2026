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
  public static final double SHOOTER_MOI = 0.01;
  public static final double SHOOTER_GEAR_RATIO = 1.0;
  // verify the following with CAD, not sure if these are correct:
  public static final double EXIT_HEIGHT = .45; // meters
  public static final double LAUNCH_ANGLE = 65; // degrees
  public static final double WHEEL_DIAMETER = Units.inchesToMeters(4); // meters
  public static final double SLIP_FACTOR = 0.6;
  public static final Translation2d ROBOT_TO_SHOOTER = new Translation2d(2.0, 0.0);

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

  // sim
  public static final double BPS = 15.0;

  // shot calculation
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
    SHOT_CALC.loadLUTEntry(2.0, 2005.859375, 0.7685994994437195);
    SHOT_CALC.loadLUTEntry(2.05, 2019.53125, 0.7831516073249437);
    SHOT_CALC.loadLUTEntry(2.1, 2033.203125, 0.7975423493214319);
    SHOT_CALC.loadLUTEntry(2.15, 2046.875, 0.8117751630755716);
    SHOT_CALC.loadLUTEntry(2.2, 2060.546875, 0.8258534145262492);
    SHOT_CALC.loadLUTEntry(2.25, 2074.21875, 0.8397803700491269);
    SHOT_CALC.loadLUTEntry(2.3, 2087.890625, 0.8535591992716633);
    SHOT_CALC.loadLUTEntry(2.35, 2101.5625, 0.8671929778258298);
    SHOT_CALC.loadLUTEntry(2.4, 2122.0703125, 0.8779047367544796);
    SHOT_CALC.loadLUTEntry(2.45, 2142.578125, 0.8884476652676296);
    SHOT_CALC.loadLUTEntry(2.5, 2156.25, 0.9016172605106972);
    SHOT_CALC.loadLUTEntry(2.55, 2169.921875, 0.9146546532560085);
    SHOT_CALC.loadLUTEntry(2.6, 2183.59375, 0.9275624907556614);
    SHOT_CALC.loadLUTEntry(2.65, 2197.265625, 0.94034334515593);
    SHOT_CALC.loadLUTEntry(2.7, 2217.7734375, 0.9501225090224119);
    SHOT_CALC.loadLUTEntry(2.75, 2231.4453125, 0.9626367082572761);
    SHOT_CALC.loadLUTEntry(2.8, 2251.953125, 0.9721333543330255);
    SHOT_CALC.loadLUTEntry(2.85, 2265.625, 0.9843926792943901);
    SHOT_CALC.loadLUTEntry(2.9, 2279.296875, 0.9965379802347796);
    SHOT_CALC.loadLUTEntry(2.95, 2292.96875, 1.0085714319297072);
    SHOT_CALC.loadLUTEntry(3.0, 2313.4765625, 1.017542311079128);
    SHOT_CALC.loadLUTEntry(3.05, 2327.1484375, 1.029341420338808);
    SHOT_CALC.loadLUTEntry(3.1, 2347.65625, 1.0380674545689461);
    SHOT_CALC.loadLUTEntry(3.15, 2361.328125, 1.0496421556575664);
    SHOT_CALC.loadLUTEntry(3.2, 2375.0, 1.0611159372657106);
    SHOT_CALC.loadLUTEntry(3.25, 2388.671875, 1.0724906752089214);
    SHOT_CALC.loadLUTEntry(3.3, 2409.1796875, 1.080757209959422);
    SHOT_CALC.loadLUTEntry(3.35, 2422.8515625, 1.0919248916106916);
    SHOT_CALC.loadLUTEntry(3.4, 2436.5234375, 1.1029991769787897);
    SHOT_CALC.loadLUTEntry(3.45, 2457.03125, 1.1109473036939332);
    SHOT_CALC.loadLUTEntry(3.5, 2470.703125, 1.1218266359139295);
    SHOT_CALC.loadLUTEntry(3.55, 2484.375, 1.1326178046196311);
    SHOT_CALC.loadLUTEntry(3.6, 2498.046875, 1.1433223445646767);
    SHOT_CALC.loadLUTEntry(3.65, 2518.5546875, 1.1508750039584037);
    SHOT_CALC.loadLUTEntry(3.7, 2532.2265625, 1.1613990642920495);
    SHOT_CALC.loadLUTEntry(3.75, 2545.8984375, 1.1718412273534315);
    SHOT_CALC.loadLUTEntry(3.8, 2566.40625, 1.1791197833238634);
    SHOT_CALC.loadLUTEntry(3.85, 2580.078125, 1.1893915747521862);
    SHOT_CALC.loadLUTEntry(3.9, 2593.75, 1.1995858772027854);
    SHOT_CALC.loadLUTEntry(3.95, 2607.421875, 1.2097039983189457);
    SHOT_CALC.loadLUTEntry(4.0, 2627.9296875, 1.2166403674021464);
    SHOT_CALC.loadLUTEntry(4.05, 2641.6015625, 1.2266002712054564);
    SHOT_CALC.loadLUTEntry(4.1, 2655.2734375, 1.236488001260899);
    SHOT_CALC.loadLUTEntry(4.15, 2668.9453125, 1.246304741512832);
    SHOT_CALC.loadLUTEntry(4.2, 2689.453125, 1.2529252509322695);
    SHOT_CALC.loadLUTEntry(4.25, 2703.125, 1.2625948538174805);
    SHOT_CALC.loadLUTEntry(4.3, 2716.796875, 1.272197105746051);
    SHOT_CALC.loadLUTEntry(4.35, 2730.46875, 1.2817331192237265);
    SHOT_CALC.loadLUTEntry(4.4, 2750.9765625, 1.2880616899283406);
    SHOT_CALC.loadLUTEntry(4.45, 2764.6484375, 1.297460655901625);
    SHOT_CALC.loadLUTEntry(4.5, 2778.3203125, 1.3067966957231287);
    SHOT_CALC.loadLUTEntry(4.55, 2791.9921875, 1.3160708241261396);
    SHOT_CALC.loadLUTEntry(4.6, 2805.6640625, 1.3252840348926453);
    SHOT_CALC.loadLUTEntry(4.65, 2826.171875, 1.3312755304643142);
    SHOT_CALC.loadLUTEntry(4.7, 2839.84375, 1.340362988132718);
    SHOT_CALC.loadLUTEntry(4.75, 2853.515625, 1.3493925104414346);
    SHOT_CALC.loadLUTEntry(4.8, 2867.1875, 1.35836499468063);
    SHOT_CALC.loadLUTEntry(4.85, 2887.6953125, 1.3641106772471896);
    SHOT_CALC.loadLUTEntry(4.9, 2901.3671875, 1.372965693301732);
    SHOT_CALC.loadLUTEntry(4.95, 2915.0390625, 1.3817664479266358);
    SHOT_CALC.loadLUTEntry(5.0, 2928.7109375, 1.3905137444156321);
    SHOT_CALC.loadLUTEntry(5.05, 2942.3828125, 1.3992083863940699);
    SHOT_CALC.loadLUTEntry(5.1, 2956.0546875, 1.4078511850925717);
    SHOT_CALC.loadLUTEntry(5.15, 2976.5625, 1.4132553228297393);
    SHOT_CALC.loadLUTEntry(5.2, 2990.234375, 1.421791693635067);
    SHOT_CALC.loadLUTEntry(5.25, 3003.90625, 1.4302786684737565);
    SHOT_CALC.loadLUTEntry(5.3, 3017.578125, 1.4387169740264758);
    SHOT_CALC.loadLUTEntry(5.35, 3031.25, 1.4471073251687323);
    SHOT_CALC.loadLUTEntry(5.4, 3048.33984375, 1.453850856906417);
    SHOT_CALC.loadLUTEntry(5.45, 3065.4296875, 1.4605475441860452);
    SHOT_CALC.loadLUTEntry(5.5, 3079.1015625, 1.4687940126116819);
    SHOT_CALC.loadLUTEntry(5.55, 3092.7734375, 1.4769954147111544);
    SHOT_CALC.loadLUTEntry(5.6, 3106.4453125, 1.4851524372266038);
    SHOT_CALC.loadLUTEntry(5.65, 3120.1171875, 1.4932656954036425);
    SHOT_CALC.loadLUTEntry(5.7, 3133.7890625, 1.5013358241531103);
    SHOT_CALC.loadLUTEntry(5.75, 3154.296875, 1.506156353968665);
    SHOT_CALC.loadLUTEntry(5.8, 3167.96875, 1.5141385357985802);
    SHOT_CALC.loadLUTEntry(5.85, 3181.640625, 1.5220795339626996);
    SHOT_CALC.loadLUTEntry(5.9, 3195.3125, 1.5299799387847175);
    SHOT_CALC.loadLUTEntry(5.95, 3208.984375, 1.537840344010754);
    SHOT_CALC.loadLUTEntry(6.0, 3222.65625, 1.5456613008135867);
  }
}
