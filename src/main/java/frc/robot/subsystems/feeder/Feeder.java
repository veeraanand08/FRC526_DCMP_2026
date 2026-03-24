// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  public void enableIndexer() {
    io.setIndexerRPS(FeederConstants.INDEXER_RPS);
  }

  public void enableKicker() {
    io.setKickerRPS(FeederConstants.KICKER_RPS);
  }

  public boolean hasSpunUp(){
        return inputs.kickerVelocityRPS > FeederConstants.KICKER_RPS - 1.5;
    }


  public void stop() {
    io.stopIndexer();
    io.stopKicker();
  }
}