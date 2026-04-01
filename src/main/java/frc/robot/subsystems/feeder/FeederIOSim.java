package frc.robot.subsystems.feeder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FeederIOSim implements FeederIO {
  private final FlywheelSim indexerSim;
  private final FlywheelSim kickerSim;

  private final PIDController kickerPID;
  private final PIDController indexerPID;

  private double indexerVolts;
  private double kickerVolts;

  private boolean isClosedLoopIndexer;
  private boolean isClosedLoopKicker;

  public FeederIOSim() {

    indexerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                FeederConstants.INDEXER_MOI,
                FeederConstants.KICKER_GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    kickerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                FeederConstants.KICKER_MOI,
                FeederConstants.KICKER_GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    kickerPID =
        new PIDController(
            FeederConstants.KICKER_KP * 125.0,
            FeederConstants.KICKER_KI,
            FeederConstants.KICKER_KD);
    indexerPID =
        new PIDController(
            FeederConstants.INDEXER_KP * 125.0,
            FeederConstants.INDEXER_KI,
            FeederConstants.INDEXER_KD);
  }

  public void updateInputs(FeederIOInputs inputs) {
    if (isClosedLoopKicker) {
      kickerVolts = kickerPID.calculate(kickerSim.getAngularVelocityRPM());
    }
    if (isClosedLoopIndexer) {
      indexerVolts = indexerPID.calculate(indexerSim.getAngularVelocityRPM());
    }

    kickerSim.setInput(kickerVolts);
    indexerSim.setInput(indexerVolts);

    indexerSim.update(0.02);
    kickerSim.update(0.02);

    inputs.indexerConnected = true;
    inputs.indexerAppliedVolts = indexerVolts;
    inputs.indexerCurrentAmps = indexerSim.getCurrentDrawAmps();
    inputs.indexerVelocityRPS = indexerSim.getAngularVelocityRPM() / 60.0;

    inputs.kickerConnected = true;
    inputs.kickerAppliedVolts = kickerVolts;
    inputs.kickerCurrentAmps = kickerSim.getCurrentDrawAmps();
    inputs.kickerVelocityRPS = kickerSim.getAngularVelocityRPM() / 60.0;
  }

  public void setIndexerRPS(double rps) {
    isClosedLoopIndexer = true;
    indexerPID.setSetpoint(rps * 60.0);
  }

  @Override
  public void setKickerRPS(double rps) {
    isClosedLoopKicker = true;
    kickerPID.setSetpoint(rps * 60.0);
  }

  @Override
  public void stopIndexer() {
    isClosedLoopIndexer = false;
    indexerVolts = 0.0;
  }

  @Override
  public void stopKicker() {
    isClosedLoopKicker = false;
    kickerVolts = 0.0;
  }
}
