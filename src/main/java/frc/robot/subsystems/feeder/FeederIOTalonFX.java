package frc.robot.subsystems.feeder;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;

public class FeederIOTalonFX implements FeederIO {
    private final TalonFX spindexer;
    private final TalonFX kicker;

    private final VelocityVoltage spindexerPid;
    private final VelocityVoltage kickerPid;

    // inputs from spindexer
    private final StatusSignal<Voltage> spindexerVoltage;
    private final StatusSignal<Current> spindexerCurrent;
    private final StatusSignal<AngularVelocity> spindexerVelocity;

    // inputs from kicker
    private final StatusSignal<Voltage> kickerVoltage;
    private final StatusSignal<Current> kickerCurrent;
    private final StatusSignal<AngularVelocity> kickerVelocity;

    public FeederIOTalonFX() {
        spindexer = new TalonFX(CANConstants.spindexer, Constants.SUPERSTRUCTURE_CAN_BUS);
        kicker = new TalonFX(CANConstants.kicker, Constants.SUPERSTRUCTURE_CAN_BUS);

        spindexerPid = new VelocityVoltage(0);
        kickerPid = new VelocityVoltage(0);

        TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
        TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

        spindexerConfig.MotorOutput.Inverted = FeederConstants.SPINDEXER_INVERTED;
        spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        spindexerConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.SPINDEXER_STATOR_LIMIT;
        spindexerConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.SPINDEXER_SUPPLY_LIMIT;
        spindexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        spindexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        spindexerConfig.Slot0.kP = FeederConstants.SPINDEXER_P;
        spindexerConfig.Slot0.kI = FeederConstants.SPINDEXER_I;
        spindexerConfig.Slot0.kD = FeederConstants.SPINDEXER_D;

        kickerConfig.MotorOutput.Inverted = FeederConstants.KICKER_INVERTED;
        kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kickerConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.KICKER_STATOR_LIMIT;
        kickerConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.KICKER_SUPPLY_LIMIT;
        kickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        kickerConfig.Slot0.kP = FeederConstants.KICKER_P;
        kickerConfig.Slot0.kI = FeederConstants.KICKER_I;
        kickerConfig.Slot0.kD = FeederConstants.KICKER_D;

        tryUntilOk(5, () -> spindexer.getConfigurator().apply(spindexerConfig));
        tryUntilOk(5, () -> spindexer.getConfigurator().apply(kickerConfig));

        spindexerVoltage = spindexer.getMotorVoltage();
        spindexerCurrent = spindexer.getStatorCurrent();
        spindexerVelocity = spindexer.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            spindexerVoltage,
            spindexerCurrent,
            spindexerVelocity);

        kickerVoltage = kicker.getMotorVoltage();
        kickerCurrent = kicker.getStatorCurrent();
        kickerVelocity = kicker.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            kickerVoltage,
            kickerCurrent,
            kickerVelocity);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        var spindexerStatus = BaseStatusSignal.refreshAll(spindexerVoltage, spindexerCurrent, spindexerVelocity);
        var kickerStatus = BaseStatusSignal.refreshAll(kickerVoltage, kickerCurrent, kickerVelocity);

        inputs.spindexerConnected = spindexerStatus.isOK();
        inputs.spindexerAppliedVolts = spindexerVoltage.getValueAsDouble();
        inputs.spindexerCurrentAmps = spindexerCurrent.getValueAsDouble();
        inputs.spindexerVelocityRPS = spindexerVelocity.getValueAsDouble();

        inputs.kickerConnected = kickerStatus.isOK();
        inputs.kickerAppliedVolts = kickerVoltage.getValueAsDouble();
        inputs.kickerCurrentAmps = kickerCurrent.getValueAsDouble();
        inputs.kickerVelocityRPS = kickerVelocity.getValueAsDouble();
    }

    @Override
    public void setSpindexerRPS(double rps) {
        spindexer.setControl(spindexerPid.withVelocity(rps));
    }

    @Override
    public void setKickerRPS(double rps) {
        kicker.setControl(kickerPid.withVelocity(rps));
    }

    @Override
    public void stopSpindexer() {
        spindexer.stopMotor();
    }

    @Override
    public void stopKicker() {
        kicker.stopMotor();
    }
}
