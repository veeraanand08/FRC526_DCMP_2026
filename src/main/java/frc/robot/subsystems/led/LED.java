package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ShiftTimer;
import frc.robot.util.subsystems.ExtendedSubsystem;
import frc.robot.util.subsystems.RobotStateHandler;
import org.littletonrobotics.junction.Logger;

public class LED extends ExtendedSubsystem {
  private enum LEDState {
    DISABLED,
    WARN,
    ACTIVE,
    INTAKE,
    READY_SHOOT,
    SHOOT,
    HUB_OFF
  }

  private static LED instance;

  public static LED getInstance() {
    if (instance == null) {
      instance = new LED(null);
      DriverStation.reportWarning(
          "LED subsystem has not been manually initialized, using default values", false);
    }
    return instance;
  }

  public static LED createInstance(Shooter shooter) {
    instance = new LED(shooter);
    return instance;
  }

  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;
  private final LEDConstants patterns;

  private LEDState state = LEDState.DISABLED;
  private boolean intake;
  private boolean shoot;

  private final Timer timer = new Timer();

  private LED(Shooter shooter) {
    led = new AddressableLED(LEDConstants.PORT);
    ledBuffer = new AddressableLEDBuffer(LEDConstants.LENGTH);
    led.setLength(ledBuffer.getLength());
    if (shooter == null) {
      patterns = new LEDConstants(() -> 1.0);
    } else {
      patterns = new LEDConstants(() -> shooter.getVelocityRPS() / shooter.getSetpointRPS());
    }

    applyPattern(patterns.disabled);
    led.start();

    Logger.recordOutput("LED/CurrentState", state);
  }

  @Override
  public void enable() {
    state = LEDState.ACTIVE;
  }

  @Override
  public void disable() {
    warn();
    timer.restart();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LED/CurrentState", state);

    if (state == LEDState.SHOOT && !ShiftTimer.instance.isHubActive()) {
      state = LEDState.HUB_OFF;
    } else if (state == LEDState.HUB_OFF && ShiftTimer.instance.isHubActive()) {
      state = LEDState.SHOOT;
    }

    if (!RobotStateHandler.isEnabled() && state == LEDState.WARN && timer.hasElapsed(3.0)) {
      timer.stop();
      state = LEDState.DISABLED;
    }

    switch (state) {
      case DISABLED -> applyPattern(patterns.disabled);
      case WARN -> applyPattern(patterns.warning);
      case ACTIVE -> applyPattern(patterns.idle);
      case INTAKE -> applyPattern(patterns.intake);
      case READY_SHOOT -> applyPattern(patterns.shooterReady);
      case SHOOT -> applyPattern(patterns.shoot);
      case HUB_OFF -> applyPattern(patterns.hubWarning);
    }
  }

  public void warn() {
    state = LEDState.WARN;
  }

  public void intake() {
    intake = true;
    if (state != LEDState.ACTIVE) return;
    state = LEDState.INTAKE;
  }

  public void shooterReady() {
    if (state == LEDState.SHOOT || state == LEDState.HUB_OFF) return;
    state = LEDState.READY_SHOOT;
  }

  public void shoot() {
    shoot = true;
    state = LEDState.SHOOT;
  }

  public void stopIntake() {
    intake = false;
    if (state == LEDState.INTAKE) state = LEDState.ACTIVE;
  }

  public void stopShoot() {
    shoot = false;
    if (intake) state = LEDState.INTAKE;
    else if (state != LEDState.READY_SHOOT) state = LEDState.ACTIVE;
  }

  private void applyPattern(LEDPattern pattern) {
    pattern.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }
}
