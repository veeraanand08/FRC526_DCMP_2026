package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.AGITATION_MID_ANGLE;
import static frc.robot.subsystems.intake.IntakeConstants.SETPOINTS;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.led.LED;
import frc.robot.util.RobotUtil;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.MotorIOTalonFX;
import frc.robot.util.io.motors.pivot.Pivot;
import frc.robot.util.io.motors.pivot.PivotIO;
import frc.robot.util.io.motors.pivot.PivotIOSim;
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
import frc.robot.util.subsystems.ExtendedSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Intake extends ExtendedSubsystem {
  public enum PivotState {
    RAISING,
    AGITATING,
    LOWERING
  }

  private PivotState pivotState = PivotState.RAISING;

  private final Roller roller;
  private final Pivot pivot;

  private final Debouncer stallDebounce = new Debouncer(0.5, Debouncer.DebounceType.kRising);
  private final Timer agitationTimer = new Timer();

  @Getter private boolean intakeRunning;

  public Intake() {
    PivotIO pivotIO =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX(
                  Constants.CANConstants.SUPERSTRUCTURE_CAN_BUS,
                  Constants.CANConstants.INTAKE_PIVOT,
                  IntakeConstants.PIVOT_CONFIG)
              .withControlRequest(
                  new MotionMagicVoltage(0).withOverrideBrakeDurNeutral(true), false)
              .withControlRequest(new PositionVoltage(0).withOverrideBrakeDurNeutral(true), true);
          case SIM -> new PivotIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.RotationalMechanismConstraints(
                  IntakeConstants.PIVOT_GEAR_RATIO,
                  SingleJointedArmSim.estimateMOI(0.5, 2),
                  0.5,
                  0,
                  IntakeConstants.MAX_ANGLE.in(Radians),
                  0),
              IntakeConstants.PIVOT_KP,
              IntakeConstants.PIVOT_KD,
              0);
          default -> new PivotIO() {};
        };
    RollerIO rollerIO =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX(
              Constants.CANConstants.SUPERSTRUCTURE_CAN_BUS,
              Constants.CANConstants.INTAKE_ROLLER,
              IntakeConstants.ROLLER_CONFIG);
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.RotationalMechanismConstraints(
                  IntakeConstants.ROLLER_GEAR_RATIO, IntakeConstants.ROLLER_MOI, 0.2, 0, 0, 0),
              IntakeConstants.ROLLER_KP * 20,
              IntakeConstants.ROLLER_KD,
              0);
          default -> new RollerIO() {};
        };

    pivot =
        new Pivot(
            "Intake/Pivot", pivotIO, IntakeConstants.PIVOT_CONFIG.CurrentLimits.StatorCurrentLimit);
    roller = new Roller("Intake/Roller", rollerIO);

    Logger.recordOutput("Intake/PivotState", pivotState.toString());
    Logger.recordOutput("Intake/Running", false);
  }

  @Override
  public void disable() {
    pivot.stop();
    roller.stop();
  }

  @Override
  public void enable() {
    pivot.stop();
    roller.stop();
  }

  @Override
  public void periodic() {
    pivot.periodic();
    roller.periodic();

    //    if (stallDebounce.calculate(
    //        pivotState == PivotState.LOWERING
    //            && pivot.inputs.appliedVoltage > 2.0
    //            && pivot.inputs.velocityDegPerSec < 1
    //            && pivot.inputs.statorCurrentAmps > 25
    //            && pivot.getPositionDeg() > 100)) {
    //      pivot.resetPosition(IntakeConstants.MAX_ANGLE);
    //    }
  }

  public void setPivotState(PivotState newState) {
    if (newState != PivotState.AGITATING) {
      pivot.runClosedLoop(SETPOINTS.get(newState));
    }
    pivotState = newState;
    Logger.recordOutput("Intake/PivotState", pivotState.toString());
  }

  public void start() {
    if (pivotState != PivotState.LOWERING) {
      setPivotState(PivotState.LOWERING);
    } else if (pivot.getPositionDeg() > 100.0) {
      pivot.runOpenLoop(0.2);
    }
    roller.runClosedLoop(IntakeConstants.ROLLER_RPS);
    LED.getInstance().intake();
    intakeRunning = true;
    Logger.recordOutput("Intake/Running", true);
  }

  public void stop() {
    roller.stop();
    if (pivot.getMode() == MotorIO.MotorIOMode.VOLTAGE_CONTROL) {
      pivot.stop();
    }
    LED.getInstance().stopIntake();
    intakeRunning = false;
    Logger.recordOutput("Intake/Running", false);
  }

  public void slowRoller() {
    roller.runClosedLoop(IntakeConstants.ROLLER_RPS_SLOW);
    Logger.recordOutput("Intake/Running", false);
  }

  public void reverseRoller() {
    roller.runClosedLoop(IntakeConstants.ROLLER_RPS_REVERSED);
    Logger.recordOutput("Intake/Running", false);
  }

  public double getPivotPosition() {
    return pivot.getPositionDeg();
  }

  public double getRollerRPS() {
    return roller.getVelocityRPS();
  }

  /**
   * Enable the intake roller. If the intake is raised, it will lower and then start.
   *
   * @return a command to run the intake
   */
  public Command intake() {
    return startEnd(this::start, this::stop);
  }

  /**
   * Toggle the intake roller. If the intake is raised, it will lower and then start.
   *
   * @return a command to run the intake
   */
  public Command deploy() {
    return runOnce(this::start);
  }

  /**
   * Reverse the intake roller while held
   *
   * @return a command to reverse the intake
   */
  public Command reverse() {
    return startEnd(this::reverseRoller, this::stop);
  }

  /**
   * Agitate balls in hopper
   *
   * @return a command to agitate the intake
   */
  public Command agitate(Feeder feeder) {
    return new FunctionalCommand(
        () -> {
          if (!feeder.isEnabledForShooting()) {
            feeder.agitate();
          }
          slowRoller();
          setPivotState(PivotState.AGITATING);
          agitationTimer.restart();
        },
        () -> {
          double targetAngle =
              IntakeConstants.AGITATION_HALF_AMPLITUDE
                      * -Math.cos(IntakeConstants.AGITATION_RAD_PER_SEC * agitationTimer.get())
                  + AGITATION_MID_ANGLE;
          pivot.runClosedLoop(Degrees.of(targetAngle), true);
        },
        interrupted -> {
          if (!feeder.isEnabledForShooting()) {
            feeder.stop();
          }
          setPivotState(PivotState.LOWERING);
          stop();
          agitationTimer.stop();
        },
        () -> false,
        this);
  }

  /**
   * Stop the roller motor and bring the intake back up
   *
   * @return a command to reset the intake back to starting position
   */
  public Command stow() {
    return runOnce(
        () -> {
          setPivotState(PivotState.RAISING);
          stop();
        });
  }

  public Command markIntakeLowered() {
    RobotUtil.RumbleRequest rumble = new RobotUtil.RumbleRequest(0.75, -1);
    return startEnd(
        () -> {
          RobotUtil.requestOperatorRumble(rumble);
          pivot.resetPosition(IntakeConstants.MAX_ANGLE);
        },
        () -> RobotUtil.stopOperatorRumble(rumble));
  }

  public Command markIntakeRaised() {
    return runOnce(() -> pivot.resetPosition(Rotations.zero()));
  }
}
