package frc.robot.subsystems.feeder;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANConstants;

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX indexer;
  private final TalonFX kicker;

  private final VelocityVoltage indexerPid;
  private final VelocityVoltage kickerPid;

  // inputs from spindexer
  private final StatusSignal<Voltage> indexerVoltage;
  private final StatusSignal<Current> indexerCurrent;
  private final StatusSignal<AngularVelocity> indexerVelocity;

  // inputs from kicker
  private final StatusSignal<Voltage> kickerVoltage;
  private final StatusSignal<Current> kickerCurrent;
  private final StatusSignal<AngularVelocity> kickerVelocity;

  public FeederIOTalonFX() {
    indexer = new TalonFX(CANConstants.INDEXER, CANConstants.SUPERSTRUCTURE_CAN_BUS);
    kicker = new TalonFX(CANConstants.KICKER, CANConstants.SUPERSTRUCTURE_CAN_BUS);

    indexerPid = new VelocityVoltage(0);
    kickerPid = new VelocityVoltage(0);

    TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
    TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

    indexerConfig.MotorOutput.Inverted = FeederConstants.INDEXER_INVERTED;
    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.INDEXER_STATOR_LIMIT;
    indexerConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.INDEXER_SUPPLY_LIMIT;
    indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    indexerConfig.Slot0.kP = FeederConstants.INDEXER_KP;
    indexerConfig.Slot0.kI = FeederConstants.INDEXER_KI;
    indexerConfig.Slot0.kD = FeederConstants.INDEXER_KD;
    indexerConfig.Slot0.kS = FeederConstants.INDEXER_KS;
    indexerConfig.Slot0.kV =  FeederConstants.INDEXER_KV;

    kickerConfig.MotorOutput.Inverted = FeederConstants.KICKER_INVERTED;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.KICKER_STATOR_LIMIT;
    kickerConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.KICKER_SUPPLY_LIMIT;
    kickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    kickerConfig.Slot0.kP = FeederConstants.KICKER_KP;
    kickerConfig.Slot0.kI = FeederConstants.KICKER_KI;
    kickerConfig.Slot0.kD = FeederConstants.KICKER_KD;
    kickerConfig.Slot0.kS = FeederConstants.KICKER_KS;
    kickerConfig.Slot0.kV =  FeederConstants.KICKER_KV;

    tryUntilOk(5, () -> indexer.getConfigurator().apply(indexerConfig));
    tryUntilOk(5, () -> kicker.getConfigurator().apply(kickerConfig));

    indexerVoltage = indexer.getMotorVoltage();
    indexerCurrent = indexer.getStatorCurrent();
    indexerVelocity = indexer.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, indexerVoltage, indexerCurrent, indexerVelocity);

    kickerVoltage = kicker.getMotorVoltage();
    kickerCurrent = kicker.getStatorCurrent();
    kickerVelocity = kicker.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, kickerVoltage, kickerCurrent, kickerVelocity);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    var indexerStatus =
        BaseStatusSignal.refreshAll(indexerVoltage, indexerCurrent, indexerVelocity);
    var kickerStatus = BaseStatusSignal.refreshAll(kickerVoltage, kickerCurrent, kickerVelocity);

    inputs.indexerConnected = indexerStatus.isOK();
    inputs.indexerAppliedVolts = indexerVoltage.getValueAsDouble();
    inputs.indexerCurrentAmps = indexerCurrent.getValueAsDouble();
    inputs.indexerVelocityRPS = indexerVelocity.getValueAsDouble();

    inputs.kickerConnected = kickerStatus.isOK();
    inputs.kickerAppliedVolts = kickerVoltage.getValueAsDouble();
    inputs.kickerCurrentAmps = kickerCurrent.getValueAsDouble();
    inputs.kickerVelocityRPS = kickerVelocity.getValueAsDouble();
  }

  @Override
  public void setIndexerRPS(double rps) {
    indexer.setControl(indexerPid.withVelocity(rps));
  }

  @Override
  public void setKickerRPS(double rps) {
    kicker.setControl(kickerPid.withVelocity(rps));
  }

  @Override
  public void stopIndexer() {
    indexer.stopMotor();
  }

  @Override
  public void stopKicker() {
    kicker.stopMotor();
  }
}
