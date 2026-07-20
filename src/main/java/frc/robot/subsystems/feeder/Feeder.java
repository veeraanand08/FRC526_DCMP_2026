// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
import frc.robot.util.io.motors.roller.RollerIOTalonFX;
import lombok.Getter;

public class Feeder extends SubsystemBase {
  private final Roller indexer;
  private final Roller kicker;

  @Getter private boolean enabledForShooting;

  public Feeder() {
    RollerIO indexerIO =
        switch (Constants.currentMode) {
          case REAL -> new RollerIOTalonFX(
              Constants.CANConstants.SUPERSTRUCTURE_CAN_BUS,
              Constants.CANConstants.INDEXER,
              FeederConstants.INDEXER_CONFIG);
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.MechanismConstraints(1, FeederConstants.INDEXER_MOI, 0.2, 0, 0, 0),
              FeederConstants.INDEXER_KP,
              FeederConstants.INDEXER_KD,
              0);
          default -> new RollerIO() {};
        };
    RollerIO kickerIO =
        switch (Constants.currentMode) {
          case REAL -> new RollerIOTalonFX(
              Constants.CANConstants.SUPERSTRUCTURE_CAN_BUS,
              Constants.CANConstants.KICKER,
              FeederConstants.KICKER_CONFIG);
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.MechanismConstraints(1, FeederConstants.KICKER_MOI, 0.2, 0, 0, 0),
              FeederConstants.KICKER_KP,
              FeederConstants.KICKER_KD,
              0);
          default -> new RollerIO() {};
        };

    indexer = new Roller("Indexer", indexerIO);
    kicker = new Roller("Kicker", kickerIO);
  }

  @Override
  public void periodic() {
    indexer.periodic();
    kicker.periodic();
  }

  public void start() {
    enabledForShooting = true;
    indexer.runClosedLoop(FeederConstants.INDEXER_RPS);
    kicker.runClosedLoop(FeederConstants.KICKER_RPS);
  }

  public void reverse() {
    indexer.runClosedLoop(-FeederConstants.INDEXER_RPS * 0.7);
    kicker.runClosedLoop(-FeederConstants.KICKER_RPS * 0.7);
    enabledForShooting = false;
  }

  public void agitate() {
    indexer.runClosedLoop(FeederConstants.INDEXER_RPS * 0.25);
    kicker.runClosedLoop(-FeederConstants.KICKER_RPS * 0.25);
  }

  public void stop() {
    indexer.stop();
    kicker.stop();
    enabledForShooting = false;
  }

  public boolean hasSpunUp() {
    return kicker.getVelocityRPS() > FeederConstants.KICKER_RPS - 1.5;
  }

  public Command feed() {
    return startEnd(this::start, this::stop)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  public Command burst() {
    return startEnd(this::start, this::stop);
  }

  public Command reverseCommand() {
    return startEnd(this::reverse, this::stop);
  }
}
