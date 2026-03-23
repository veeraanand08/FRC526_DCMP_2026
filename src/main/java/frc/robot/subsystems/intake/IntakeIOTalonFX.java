package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANConstants;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX roller;
    private final TalonFX pivot;

    private final CANcoder pivotEncoder;

    private final VelocityVoltage rollerPid;
    private final PositionVoltage pivotPid;

    private final StatusSignal<Voltage> pivotVoltage;
    private final StatusSignal<Current> pivotCurrent;
    private final StatusSignal<Angle> pivotAngle;
    private final StatusSignal<AngularVelocity> pivotVelocity;

    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerCurrent;
    private final StatusSignal<AngularVelocity> rollerVelocity;

    private final StatusSignal<Angle> encoderAngle;
    
    public IntakeIOTalonFX() {
        roller = new TalonFX(CANConstants.roller, CANConstants.SUPERSTRUCTURE_CAN_BUS);
        pivot = new TalonFX(CANConstants.pivot, CANConstants.SUPERSTRUCTURE_CAN_BUS);
        pivotEncoder = new CANcoder(CANConstants.encoder,  CANConstants.SUPERSTRUCTURE_CAN_BUS);

        rollerPid = new VelocityVoltage(0);
        pivotPid = new PositionVoltage(0).withOverrideBrakeDurNeutral(true);

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        rollerConfig.MotorOutput.Inverted = IntakeConstants.ROLLER_INVERTED;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_LIMIT;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_SUPPLY_LIMIT;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        rollerConfig.Slot0.kP = IntakeConstants.ROLLER_P;
        rollerConfig.Slot0.kI = IntakeConstants.ROLLER_I;
        rollerConfig.Slot0.kD = IntakeConstants.ROLLER_D;

        pivotConfig.MotorOutput.Inverted = IntakeConstants.PIVOT_INVERTED;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        pivotConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.PIVOT_STATOR_LIMIT;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.PIVOT_SUPPLY_LIMIT;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        pivotConfig.Feedback.RotorToSensorRatio = IntakeConstants.PIVOT_GEAR_RATIO;

        pivotConfig.Slot0.kP = IntakeConstants.PIVOT_P;
        pivotConfig.Slot0.kI = IntakeConstants.PIVOT_I;
        pivotConfig.Slot0.kD = IntakeConstants.PIVOT_D;

        tryUntilOk(5, () -> pivot.getConfigurator().apply(pivotConfig));
        tryUntilOk(5, () -> roller.getConfigurator().apply(rollerConfig));

        rollerVoltage = roller.getMotorVoltage();
        rollerCurrent =  roller.getStatorCurrent();
        rollerVelocity = roller.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            rollerVoltage,
            rollerCurrent,
            rollerVelocity);

        pivotVoltage = pivot.getMotorVoltage();
        pivotCurrent =  pivot.getStatorCurrent();
        pivotAngle = pivot.getPosition();
        pivotVelocity = pivot.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            pivotVoltage,
            pivotCurrent,
            pivotAngle,
            pivotVelocity);

        encoderAngle = pivotEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, encoderAngle);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        var pivotStatus = BaseStatusSignal.refreshAll(rollerVoltage, rollerCurrent, rollerVelocity);
        var rollerStatus = BaseStatusSignal.refreshAll(pivotVoltage, pivotCurrent, pivotAngle);
        var encoderStatus = BaseStatusSignal.refreshAll(encoderAngle);

        inputs.pivotConnected = pivotStatus.isOK();
        inputs.pivotAppliedVolts = pivotVoltage.getValueAsDouble();
        inputs.pivotCurrentAmps = pivotCurrent.getValueAsDouble();
        inputs.pivotPositionDeg = Units.rotationsToDegrees(pivotAngle.getValueAsDouble());
        inputs.pivotVelocityRPS = pivotVelocity.getValueAsDouble();

        inputs.rollerConnected = rollerStatus.isOK();
        inputs.rollerAppliedVolts = rollerVoltage.getValueAsDouble();
        inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
        inputs.rollerVelocityRPS = rollerVelocity.getValueAsDouble();

        inputs.encoderConnected = encoderStatus.isOK();
        inputs.encoderPositionDeg = Units.rotationsToDegrees(encoderAngle.getValueAsDouble());
    }

    @Override
    public void setPivotSetpoint(double deg) {
        pivot.setControl(pivotPid.withPosition(Units.degreesToRotations(deg)));
    }

    @Override
    public void setPivotDeg(double deg) {
        
    }

    @Override
    public void setRollerRPS(double rps) {
        roller.setControl(rollerPid.withVelocity(rps));
    }

    @Override
    public void stopPivot() {
        pivot.stopMotor();
    }

    @Override
    public void stopRoller() {
        roller.stopMotor();
    }
}
