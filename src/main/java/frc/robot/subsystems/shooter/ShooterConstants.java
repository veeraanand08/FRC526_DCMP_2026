package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.sotm.ProjectileSimulator;
import frc.robot.util.sotm.ShotCalculator;
import java.io.*;
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
  public static final double MIN_DISTANCE = 1.15;
  public static final double MAX_DISTANCE = 9.0;

  public static final double SHOOTER_KP = 0.3;
  public static final double SHOOTER_KI = 0;
  public static final double SHOOTER_KD = 0;
  public static final double SHOOTER_KS = 0;
  public static final double SHOOTER_KV = 0.12;
  public static final TalonFXConfiguration SHOOTER_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(70)
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

  public static final double SIM_BPS = 12.5;

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
    System.out.println("Finished in " + elapsedMs + " ms");

    // save to file
    File lutFile = new File(Filesystem.getDeployDirectory(), "shooter-data.csv");
    try (PrintWriter writer = new PrintWriter(new FileWriter(lutFile))) {
      writer.println("distance,rpm,tof");

      for (var entry : lut.entries()) {
        if (entry.reachable()) {
          writer.println(entry.distanceM() + "," + entry.rpm() + "," + entry.tof());
        }
      }
      System.out.println("LUT successfully saved to " + lutFile.getAbsolutePath());
    } catch (IOException e) {
      System.err.println("Failed to write LUT to file: " + e.getMessage());
    }

    System.out.println(
        "\nShutting down, remove call to ShooterConstants.generateLookupTable() to run robot.\n");
    System.exit(0);
  }

  public static void loadShooterData() {
    File lutFile = new File(Filesystem.getDeployDirectory(), "shooter-data.csv");

    if (!lutFile.exists()) {
      System.out.println("LUT file not found. Generating a new one...");
      generateLookupTable();
    }

    try (BufferedReader br = new BufferedReader(new FileReader(lutFile))) {
      System.out.println("Loading LUT from: " + lutFile.getAbsolutePath());

      // Skip the header row
      br.readLine();

      String line;
      while ((line = br.readLine()) != null) {
        // Split the line by commas
        String[] tokens = line.split(",");
        if (tokens.length < 3) continue;

        try {
          double distance = Double.parseDouble(tokens[0]);
          double rpm = Double.parseDouble(tokens[1]);
          double tof = Double.parseDouble(tokens[2]);
          SHOT_CALC.loadLUTEntry(distance, rpm, tof);
        } catch (NumberFormatException e) {
          // Handle parsing errors for bad lines
          System.err.println("Skipping malformed CSV line: " + line);
        }
      }

      System.out.println("LUT successfully loaded into memory.");
    } catch (IOException e) {
      System.err.println("Critical error loading LUT file: " + e.getMessage());
    }
  }
}
