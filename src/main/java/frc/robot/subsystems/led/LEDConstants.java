package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;
import java.util.function.DoubleSupplier;

public final class LEDConstants {
  public static final int PORT = 9;
  public static final int LENGTH = 60; // placeholder
  public static final Distance SPACING = Meters.of(1.0 / 60);

  public final LEDPattern disabled = scrollingEye(0.3, 1, Color.kGreen, Color.kGold);

  public final LEDPattern warning = LEDPattern.solid(Color.kRed).breathe(Seconds.of(0.5));

  public final LEDPattern idle =
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kGold)
          .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), SPACING);

  public final LEDPattern intake = LEDPattern.solid(Color.kCoral);

  public final LEDPattern shooterReady = LEDPattern.solid(Color.kGreen).breathe(Seconds.of(0.5));

  public final LEDPattern shoot;

  public final LEDPattern hubWarning;

  public LEDConstants(DoubleSupplier shooterMask) {
    shoot = LEDPattern.solid(Color.kGold).mask(LEDPattern.progressMaskLayer(shooterMask));
    hubWarning = shoot.breathe(Seconds.of(0.5));
  }

  public static LEDPattern scrollingEye(double windowSize, double speedMPS, Color... colors) {
    Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, windowSize, Color.kBlack);
    LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colors);
    LEDPattern mask =
        LEDPattern.steps(maskSteps).scrollAtAbsoluteSpeed(MetersPerSecond.of(speedMPS), SPACING);
    return base.mask(mask);
  }
}
