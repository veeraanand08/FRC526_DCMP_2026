package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANConstants;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX topLeft; // leader

  private final TalonFX bottomLeft;
  private final TalonFX topRight;
  private final TalonFX bottomRight;

  private final VelocityVoltage shooterPid;

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> topLeftTemp;
  private final StatusSignal<Temperature> bottomLeftTemp;
  private final StatusSignal<Temperature> topRightTemp;
  private final StatusSignal<Temperature> bottomRightTemp;

  public ShooterIOTalonFX() {
    topLeft = new TalonFX(CANConstants.SHOOTER_TOP_LEFT, CANConstants.SUPERSTRUCTURE_CAN_BUS);
    bottomLeft = new TalonFX(CANConstants.SHOOTER_BOTTOM_LEFT, CANConstants.SUPERSTRUCTURE_CAN_BUS);

    topRight = new TalonFX(CANConstants.SHOOTER_TOP_RIGHT, CANConstants.SUPERSTRUCTURE_CAN_BUS);
    bottomRight =
        new TalonFX(CANConstants.SHOOTER_BOTTOM_RIGHT, CANConstants.SUPERSTRUCTURE_CAN_BUS);

    shooterPid = new VelocityVoltage(0);

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    shooterConfig.MotorOutput.Inverted = ShooterConstants.SHOOTER_LEFT_INVERTED;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.SHOOTER_STATOR_LIMIT;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_SUPPLY_LIMIT;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    shooterConfig.Slot0.kP = ShooterConstants.SHOOTER_KP;
    shooterConfig.Slot0.kI = ShooterConstants.SHOOTER_KI;
    shooterConfig.Slot0.kD = ShooterConstants.SHOOTER_KD;
    shooterConfig.Slot0.kS = ShooterConstants.SHOOTER_KS;
    shooterConfig.Slot0.kV = ShooterConstants.SHOOTER_KV;

    tryUntilOk(5, () -> topLeft.getConfigurator().apply(shooterConfig));
    tryUntilOk(5, () -> bottomLeft.getConfigurator().apply(shooterConfig));
    tryUntilOk(5, () -> topRight.getConfigurator().apply(shooterConfig));
    tryUntilOk(5, () -> bottomRight.getConfigurator().apply(shooterConfig));

    bottomLeft.setControl(
        new Follower(topLeft.getDeviceID(), ShooterConstants.BOTTOM_LEFT_ALIGNMENT_VALUE));
    topRight.setControl(
        new Follower(topLeft.getDeviceID(), ShooterConstants.TOP_RIGHT_ALIGNMENT_VALUE));
    bottomRight.setControl(
        new Follower(topLeft.getDeviceID(), ShooterConstants.BOTTOM_RIGHT_ALIGNMENT_VALUE));

    velocity = topLeft.getVelocity();
    appliedVolts = topLeft.getMotorVoltage();
    statorCurrent = topLeft.getStatorCurrent();
    supplyCurrent = topLeft.getSupplyCurrent();
    topLeftTemp = topLeft.getDeviceTemp();
    bottomLeftTemp = bottomLeft.getDeviceTemp();
    topRightTemp = topRight.getDeviceTemp();
    bottomRightTemp = bottomRight.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, velocity, appliedVolts, statorCurrent, supplyCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, topLeftTemp, bottomLeftTemp, topRightTemp, bottomRightTemp);
    ParentDevice.optimizeBusUtilizationForAll(topLeft, bottomLeft, topRight, bottomRight);

    PhoenixUtil.registerSignals(
        CANConstants.SUPERSTRUCTURE_CAN_BUS,
        velocity,
        appliedVolts,
        statorCurrent,
        supplyCurrent,
        topLeftTemp,
        bottomLeftTemp,
        topRightTemp,
        bottomRightTemp);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.topLeftConnected =
        BaseStatusSignal.isAllGood(
            velocity, appliedVolts, statorCurrent, supplyCurrent, topLeftTemp);
    inputs.bottomLeftConnected = BaseStatusSignal.isAllGood(bottomLeftTemp);
    inputs.topRightConnected = BaseStatusSignal.isAllGood(topRightTemp);
    inputs.bottomRightConnected = BaseStatusSignal.isAllGood(bottomRightTemp);

    inputs.velocityRPS = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();

    inputs.topLeftTempCelsius = topLeftTemp.getValueAsDouble();
    inputs.bottomLeftTempCelsius = bottomLeftTemp.getValueAsDouble();
    inputs.topRightTempCelsius = topRightTemp.getValueAsDouble();
    inputs.bottomRightTempCelsius = bottomRightTemp.getValueAsDouble();
  }

  @Override
  public void setRPS(double rps) {
    topLeft.setControl(shooterPid.withVelocity(rps));
  }

  @Override
  public void stop() {
    topLeft.stopMotor();
  }
}
