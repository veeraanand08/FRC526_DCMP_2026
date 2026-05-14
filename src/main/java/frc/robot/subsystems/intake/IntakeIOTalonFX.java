package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANConstants;
import frc.robot.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX roller;
  private final TalonFX pivot;

  private final StaticBrake brakeRequest = new StaticBrake();
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage rollerPid = new VelocityVoltage(0);
  private final PositionVoltage pivotPid = new PositionVoltage(0).withOverrideBrakeDurNeutral(true);
  private final MotionMagicVoltage pivotMotionMagic =
      new MotionMagicVoltage(0).withOverrideBrakeDurNeutral(true);

  private final StatusSignal<Voltage> pivotVoltage;
  private final StatusSignal<Current> pivotCurrent;
  private final StatusSignal<Angle> pivotAngle;
  private final StatusSignal<AngularVelocity> pivotVelocity;

  private final StatusSignal<Voltage> rollerVoltage;
  private final StatusSignal<Current> rollerCurrent;
  private final StatusSignal<AngularVelocity> rollerVelocity;

  public IntakeIOTalonFX() {
    roller = new TalonFX(CANConstants.INTAKE_ROLLER, CANConstants.SUPERSTRUCTURE_CAN_BUS);
    pivot = new TalonFX(CANConstants.INTAKE_PIVOT, CANConstants.SUPERSTRUCTURE_CAN_BUS);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    rollerConfig.MotorOutput.Inverted = IntakeConstants.ROLLER_INVERTED;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_LIMIT;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_SUPPLY_LIMIT;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rollerConfig.Slot0.kP = IntakeConstants.ROLLER_KP;
    rollerConfig.Slot0.kI = IntakeConstants.ROLLER_KI;
    rollerConfig.Slot0.kD = IntakeConstants.ROLLER_KD;
    rollerConfig.Slot0.kS = IntakeConstants.ROLLER_KS;
    rollerConfig.Slot0.kV = IntakeConstants.ROLLER_KV;

    pivotConfig.MotorOutput.Inverted = IntakeConstants.PIVOT_INVERTED;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    pivotConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.PIVOT_STATOR_LIMIT;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.PIVOT_SUPPLY_LIMIT;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    pivotConfig.Feedback.SensorToMechanismRatio = IntakeConstants.PIVOT_GEAR_RATIO;

    pivotConfig.Slot0.kP = IntakeConstants.PIVOT_KP;
    pivotConfig.Slot0.kI = IntakeConstants.PIVOT_KI;
    pivotConfig.Slot0.kD = IntakeConstants.PIVOT_KD;
    pivotConfig.Slot0.kS = IntakeConstants.PIVOT_KS;
    pivotConfig.Slot0.kV = IntakeConstants.PIVOT_KV;
    pivotConfig.Slot0.kA = IntakeConstants.PIVOT_KA;
    pivotConfig.Slot0.kG = IntakeConstants.PIVOT_KG;
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.PIVOT_CRUISE_VELOCITY;
    pivotConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.PIVOT_CRUISE_ACCELERATION;

    tryUntilOk(5, () -> pivot.getConfigurator().apply(pivotConfig));
    tryUntilOk(5, () -> roller.getConfigurator().apply(rollerConfig));

    rollerVoltage = roller.getMotorVoltage();
    rollerCurrent = roller.getStatorCurrent();
    rollerVelocity = roller.getVelocity();

    // Reset the integrated encoder
    pivot.setPosition(0.0);

    pivotVoltage = pivot.getMotorVoltage();
    pivotCurrent = pivot.getStatorCurrent();
    pivotAngle = pivot.getPosition();
    pivotVelocity = pivot.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        rollerVoltage,
        rollerCurrent,
        rollerVelocity,
        pivotVoltage,
        pivotCurrent,
        pivotAngle,
        pivotVelocity);
    ParentDevice.optimizeBusUtilizationForAll(roller, pivot);

    PhoenixUtil.registerSignals(
        CANConstants.SUPERSTRUCTURE_CAN_BUS,
        rollerVoltage,
        rollerCurrent,
        rollerVelocity,
        pivotVoltage,
        pivotCurrent,
        pivotAngle,
        pivotVelocity);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.pivotConnected =
        BaseStatusSignal.isAllGood(pivotVoltage, pivotCurrent, pivotAngle, pivotVelocity);
    inputs.pivotAppliedVolts = pivotVoltage.getValueAsDouble();
    inputs.pivotCurrentAmps = pivotCurrent.getValueAsDouble();
    inputs.pivotPositionDeg = Units.rotationsToDegrees(pivotAngle.getValueAsDouble());
    inputs.pivotVelocityRPS = pivotVelocity.getValueAsDouble();

    inputs.rollerConnected =
        BaseStatusSignal.isAllGood(rollerVoltage, rollerCurrent, rollerVelocity);
    inputs.rollerAppliedVolts = rollerVoltage.getValueAsDouble();
    inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
    inputs.rollerVelocityRPS = rollerVelocity.getValueAsDouble();
  }

  @Override
  public void setPivotOpenLoop(double output) {
    pivot.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setPivotSetpoint(double deg) {
    pivot.setControl(pivotPid.withPosition(Units.degreesToRotations(deg)));
  }

  @Override
  public void setPivotProfiled(double deg) {
    pivot.setControl(pivotMotionMagic.withPosition(Units.degreesToRotations(deg)));
  }

  @Override
  public void setRollerRPS(double rps) {
    roller.setControl(rollerPid.withVelocity(rps));
  }

  @Override
  public void stopPivot() {
    pivot.setControl(brakeRequest);
  }

  @Override
  public void stopRoller() {
    roller.stopMotor();
  }

  @Override
  public void resetPivotPosition(Angle angle) {
    pivot.setPosition(angle);
  }
}
