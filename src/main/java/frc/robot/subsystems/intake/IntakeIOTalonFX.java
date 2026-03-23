package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX pivot;
    private final TalonFX roller;

    //private final RelativeEncoder pivotEncoder; figure out what encoder team uses and add that in


    private final PositionVoltage pivotPid;
    private final VelocityVoltage rollerPid;

    private final StatusSignal<Voltage> pivotVoltage;
    private final StatusSignal<Current> pivotCurrent;
    private final StatusSignal<Angle> pivotAngle;
    private final StatusSignal<AngularVelocity> pivotVelocity;


    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerCurrent;
    private final StatusSignal<AngularVelocity> rollerVelocity;

    
    public IntakeIOTalonFX() {
        roller = new TalonFX(CANConstants.roller, Constants.SUPERSTRUCTURE_CAN_BUS);
        pivot = new TalonFX(CANConstants.pivot, Constants.SUPERSTRUCTURE_CAN_BUS);

        rollerPid = new VelocityVoltage(0);
        pivotPid = new PositionVoltage(0);

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

        pivotConfig.Feedback.SensorToMechanismRatio = 360.0; //change this later with the gear ratio probably

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
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        var pivotStatus = BaseStatusSignal.refreshAll(rollerVoltage, rollerCurrent, rollerVelocity);
        var rollerStatus = BaseStatusSignal.refreshAll(pivotVoltage, pivotCurrent, pivotAngle);


        inputs.pivotConnected = pivotStatus.isOK();
        inputs.pivotAppliedVolts = pivotVoltage.getValueAsDouble();
        inputs.pivotCurrentAmps = pivotCurrent.getValueAsDouble();
        inputs.pivotPositionDeg = pivotAngle.getValueAsDouble();
        inputs.pivotVelocityDegPerSec = pivotVelocity.getValueAsDouble();

        inputs.rollerConnected = rollerStatus.isOK();
        inputs.rollerAppliedVolts =  rollerVoltage.getValueAsDouble();
        inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
        inputs.rollerVelocityRPS = rollerVelocity.getValueAsDouble();
    }

    @Override
    public void setPivotBrake(boolean brake) {
        MotorOutputConfigs config = new MotorOutputConfigs();
        config.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        pivot.getConfigurator().apply(config);
    }

    @Override
    public void setPivotSetpoint(double deg) {
        pivot.setControl(pivotPid.withPosition(deg));
    }

    @Override
    public void setPivotDeg(double deg) { //profiled pid not done
        
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
