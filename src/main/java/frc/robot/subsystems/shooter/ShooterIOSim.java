package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim shooterSim;

  private final PIDController shooterPID;

  private double shooterVolts;
  private boolean isClosedLoopShooter;

  public ShooterIOSim() {
    shooterSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(4),
                ShooterConstants.SHOOTER_MOI,
                ShooterConstants.SHOOTER_GEAR_RATIO),
            DCMotor.getKrakenX60(4));

    shooterPID =
        new PIDController(
            ShooterConstants.SHOOTER_KP * 125.0,
            ShooterConstants.SHOOTER_KI,
            ShooterConstants.SHOOTER_KD);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    if (isClosedLoopShooter) {
      shooterVolts = shooterPID.calculate(shooterSim.getAngularVelocityRPM());
    }

    shooterSim.setInput(shooterVolts);
    shooterSim.update(0.02);

    inputs.topLeftConnected = true;
    inputs.shooterAppliedVolts = shooterVolts;
    inputs.shooterCurrentAmps = shooterSim.getCurrentDrawAmps();
    inputs.shooterVelocityRPS = shooterSim.getAngularVelocityRPM() / 60.0;

    inputs.bottomLeftConnected = true;
    inputs.topRightConnected = true;
    inputs.bottomRightConnected = true;
  }

  public void setRPS(double rps) {
    isClosedLoopShooter = true;
    shooterPID.setSetpoint(rps * 60.0);
  }

  public void stop() {
    isClosedLoopShooter = false;
    shooterVolts = 0.0;
  }
}
