// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private boolean isEnabledForShooting;

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  public void enableIndexer() {
    isEnabledForShooting = true;
    io.setIndexerRPS(FeederConstants.INDEXER_RPS);
  }

  public void slowIndexer() {
    io.setIndexerRPS(FeederConstants.INDEXER_RPS * 0.25);
  }

  public void enableKicker() {
    isEnabledForShooting = true;
    io.setKickerRPS(FeederConstants.KICKER_RPS);
  }

  public void reverseKicker() {
    io.setKickerRPS(-FeederConstants.KICKER_RPS * 0.25);
  }

  public void reverse() {
    io.setIndexerRPS(-FeederConstants.INDEXER_RPS);
    io.setKickerRPS(-FeederConstants.KICKER_RPS);
  }

  public boolean hasSpunUp() {
    return inputs.kickerVelocityRPS > FeederConstants.KICKER_RPS - 1.5;
  }

  public boolean isEnabledForShooting() {
    return isEnabledForShooting;
  }

  public void stop() {
    io.stopIndexer();
    io.stopKicker();
    isEnabledForShooting = false;
  }

  public Command burst() {
    return startEnd(
        () -> {
          enableKicker();
          enableIndexer();
        },
        this::stop);
  }

  public Command reverseCommand() {
    return startEnd(this::reverse, this::stop);
  }
}
