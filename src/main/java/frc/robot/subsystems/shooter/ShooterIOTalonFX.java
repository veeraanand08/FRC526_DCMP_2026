package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX topLeft;
    private final TalonFX bottomLeft;

    private final TalonFX topRight;
    private final TalonFX bottomRight;

    private final TalonFX hood;

    private final VelocityVoltage shooterPid;
    private final PositionVoltage hoodPid;

    private final StatusSignal<Voltage> shooterVoltage;
    private final StatusSignal<Current> shooterCurrent;
    private final StatusSignal<AngularVelocity> shooterVelocity;

    private final StatusSignal<Voltage> hoodVoltage;
    private final StatusSignal<Current> hoodCurrent;
    private final StatusSignal<Angle> hoodAngle;


    public ShooterIOTalonFX() {
        topLeft = new TalonFX(CANConstants.leftLeader, Constants.SUPERSTRUCTURE_CAN_BUS);
        bottomLeft = new TalonFX(CANConstants.leftFollower, Constants.SUPERSTRUCTURE_CAN_BUS);

        topRight = new TalonFX(CANConstants.rightLeader, Constants.SUPERSTRUCTURE_CAN_BUS);
        bottomRight = new TalonFX(CANConstants.rightFollower, Constants.SUPERSTRUCTURE_CAN_BUS);
        
        hood = new TalonFX(CANConstants.hood, Constants.SUPERSTRUCTURE_CAN_BUS);

        shooterPid = new VelocityVoltage(0);
        hoodPid = new PositionVoltage(0);

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

        shooterConfig.MotorOutput.Inverted = ShooterConstants.SHOOTER_TOP_RIGHT_INVERTED;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.SHOOTER_STATOR_LIMIT;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_SUPPLY_LIMIT;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterConfig.Slot0.kP = ShooterConstants.SHOOTER_P;
        shooterConfig.Slot0.kI = ShooterConstants.SHOOTER_I;
        shooterConfig.Slot0.kD = ShooterConstants.SHOOTER_D;

        hoodConfig.MotorOutput.Inverted = ShooterConstants.HOOD_INVERTED;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hoodConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.HOOD_STATOR_LIMIT;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.HOOD_SUPPLY_LIMIT;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        hoodConfig.Feedback.SensorToMechanismRatio = 360.0;

        hoodConfig.Slot0.kP = ShooterConstants.HOOD_P;
        hoodConfig.Slot0.kI = ShooterConstants.HOOD_I;
        hoodConfig.Slot0.kD = ShooterConstants.HOOD_D;

        tryUntilOk(5, () -> topLeft.getConfigurator().apply(shooterConfig));
        tryUntilOk(5, () -> bottomLeft.getConfigurator().apply(shooterConfig));
        tryUntilOk(5, () -> topRight.getConfigurator().apply(shooterConfig));
        tryUntilOk(5, () -> bottomRight.getConfigurator().apply(shooterConfig));
        tryUntilOk(5, () -> hood.getConfigurator().apply(hoodConfig));
        
        bottomLeft.setControl(new Follower(topLeft.getDeviceID(), ShooterConstants.BOTTOM_LEFT_ALIGNMENT_VALUE));
        topRight.setControl(new Follower(topLeft.getDeviceID(), ShooterConstants.TOP_RIGHT_ALIGNMENT_VALUE));
        bottomRight.setControl(new Follower(topLeft.getDeviceID(), ShooterConstants.BOTTOM_RIGHT_ALIGNMENT_VALUE));

        shooterVoltage = topRight.getMotorVoltage();
        shooterCurrent =  topRight.getStatorCurrent();
        shooterVelocity = topRight.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            shooterVoltage,
            shooterCurrent,
            shooterVelocity);

        hoodVoltage = hood.getMotorVoltage();
        hoodCurrent =  hood.getStatorCurrent();
        hoodAngle = hood.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            hoodVoltage,
            hoodCurrent,
            hoodAngle);


        
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        var shooterStatus = BaseStatusSignal.refreshAll(shooterVoltage, shooterCurrent, shooterVelocity);
        var hoodStatus = BaseStatusSignal.refreshAll(hoodVoltage, hoodCurrent, hoodAngle);

        inputs.topLeftConnected = shooterStatus.isOK();
        inputs.shooterAppliedVolts = shooterVoltage.getValueAsDouble();
        inputs.shooterCurrentAmps = shooterCurrent.getValueAsDouble();
        inputs.shooterVelocityRPS = shooterVelocity.getValueAsDouble();

        inputs.bottomLeftConnected = bottomLeft.isAlive();
        inputs.topRightConnected = topRight.isAlive();
        inputs.bottomRightConnected = bottomRight.isAlive();

        inputs.hoodConnected = hoodStatus.isOK();
        inputs.hoodAppliedVolts = hoodVoltage.getValueAsDouble();
        inputs.hoodCurrentAmps = hoodCurrent.getValueAsDouble();
        inputs.hoodAngle = hoodAngle.getValueAsDouble();
        
    }

    @Override
    public void setRPS(double rps) {
        topLeft.setControl(shooterPid.withVelocity(rps));
    }

    @Override
    public void setAngle(double angle){
        hood.setControl(hoodPid.withPosition(angle));
    }

    @Override
    public void stop() {
        topLeft.stopMotor();
    }
}
