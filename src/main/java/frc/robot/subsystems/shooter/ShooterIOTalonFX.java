package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
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

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX leftLeader;
    private final TalonFX leftFollower;

    private final TalonFX rightLeader;
    private final TalonFX rightFollower;

    private final TalonFX hood;

    private final VelocityVoltage leftPid;
    private final VelocityVoltage rightPid;
    private final PositionVoltage hoodPid;

    private final StatusSignal<Voltage> leftVoltage;
    private final StatusSignal<Current> leftCurrent;
    private final StatusSignal<AngularVelocity> leftVelocity;
    
    private final StatusSignal<Voltage> rightVoltage;
    private final StatusSignal<Current> rightCurrent;
    private final StatusSignal<AngularVelocity> rightVelocity;

    private final StatusSignal<Voltage> hoodVoltage;
    private final StatusSignal<Current> hoodCurrent;
    private final StatusSignal<Angle> hoodAngle;


    public ShooterIOTalonFX() {
        leftLeader = new TalonFX(CANConstants.leftLeader, Constants.SUPERSTRUCTURE_CAN_BUS);
        leftFollower = new TalonFX(CANConstants.leftFollower, Constants.SUPERSTRUCTURE_CAN_BUS);

        rightLeader = new TalonFX(CANConstants.rightLeader, Constants.SUPERSTRUCTURE_CAN_BUS);
        rightFollower = new TalonFX(CANConstants.rightFollower, Constants.SUPERSTRUCTURE_CAN_BUS);
        
        hood = new TalonFX(CANConstants.hood, Constants.SUPERSTRUCTURE_CAN_BUS);

        leftPid = new VelocityVoltage(0);
        rightPid = new VelocityVoltage(0);
        hoodPid = new PositionVoltage(0);

        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

        leftConfig.MotorOutput.Inverted = ShooterConstants.SHOOTER_RIGHT_INVERTED;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.SHOOTER_STATOR_LIMIT;
        leftConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_SUPPLY_LIMIT;
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftConfig.Slot0.kP = ShooterConstants.SHOOTER_P;
        leftConfig.Slot0.kI = ShooterConstants.SHOOTER_I;
        leftConfig.Slot0.kD = ShooterConstants.SHOOTER_D;

        rightConfig.MotorOutput.Inverted = ShooterConstants.SHOOTER_LEFT_INVERTED;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.SHOOTER_STATOR_LIMIT;
        rightConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_SUPPLY_LIMIT;
        rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        rightConfig.Slot0.kP = ShooterConstants.SHOOTER_P;
        rightConfig.Slot0.kI = ShooterConstants.SHOOTER_I;
        rightConfig.Slot0.kD = ShooterConstants.SHOOTER_D;

        hoodConfig.MotorOutput.Inverted = ShooterConstants.HOOD_INVERTED;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hoodConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.HOOD_STATOR_LIMIT;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.HOOD_SUPPLY_LIMIT;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        hoodConfig.Slot0.kP = ShooterConstants.HOOD_P;
        hoodConfig.Slot0.kI = ShooterConstants.HOOD_I;
        hoodConfig.Slot0.kD = ShooterConstants.HOOD_D;

        tryUntilOk(5, () -> leftLeader.getConfigurator().apply(leftConfig));
        tryUntilOk(5, () -> leftFollower.getConfigurator().apply(leftConfig));
        tryUntilOk(5, () -> rightLeader.getConfigurator().apply(rightConfig));
        tryUntilOk(5, () -> rightFollower.getConfigurator().apply(rightConfig));
        tryUntilOk(5, () -> hood.getConfigurator().apply(hoodConfig));

        leftVoltage = leftLeader.getMotorVoltage();
        leftCurrent =  leftLeader.getStatorCurrent();
        leftVelocity = leftLeader.getVelocity();

        rightVoltage = rightLeader.getMotorVoltage();
        rightCurrent =  rightLeader.getStatorCurrent();
        rightVelocity = rightLeader.getVelocity();

        hoodVoltage = hood.getMotorVoltage();
        hoodCurrent =  hood.getStatorCurrent();
        hoodAngle = hood.getPosition();












        leader = new SparkMax(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        follower = new SparkMax(Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

        leftMotorConfig.inverted(Constants.ShooterConstants.MOTORS_REVERSED);
        leftMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        leftMotorConfig.smartCurrentLimit(Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT);
        leftMotorConfig.voltageCompensation(12.0);
        leftMotorConfig.closedLoop.pid(Constants.ShooterConstants.SHOOTER_P, Constants.ShooterConstants.SHOOTER_I, Constants.ShooterConstants.SHOOTER_D)
                .feedForward.kV(Constants.ShooterConstants.SHOOTER_FF);

        rightMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        rightMotorConfig.smartCurrentLimit(Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT);
        rightMotorConfig.voltageCompensation(12.0);
        rightMotorConfig.follow(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR, true);

        leader.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leaderEncoder = leader.getEncoder();
        followerEncoder = follower.getEncoder();
        shooterPid = leader.getClosedLoopController();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leaderConnected = !leader.hasActiveFault();
        inputs.leaderAppliedVolts = leader.getBusVoltage() * leader.getAppliedOutput();
        inputs.leaderCurrentAmps = leader.getOutputCurrent();
        inputs.leaderCurrentRPM = leaderEncoder.getVelocity();

        inputs.followerConnected = !follower.hasActiveFault();
        inputs.followerAppliedVolts =  follower.getBusVoltage() * follower.getAppliedOutput();
        inputs.followerCurrentAmps = follower.getOutputCurrent();
        inputs.followerCurrentRPM = followerEncoder.getVelocity();
    }

    @Override
    public void set(double speed) {
        leader.set(speed);
    }

    @Override
    public void setRPM(double rpm) {
        shooterPid.setSetpoint(rpm, SparkBase.ControlType.kVelocity);
    }

    @Override
    public void stop() {
        leader.stopMotor();
    }
}
