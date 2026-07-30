package frc.robot.util;

import static frc.robot.util.RobotUtil.isRedAlliance;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class ShiftTimer {
  private enum ShiftSegment {
    TRANSITION,
    ALLIANCE,
    ENDGAME
  }

  public static final ShiftTimer instance = new ShiftTimer();

  private final Timer timer;
  private ShiftSegment currentSegment;
  private int allianceShiftNum;
  @Getter private boolean isHubActive;
  @Getter private double timeRemaining;

  // rumble
  private static final RobotUtil.RumbleRequest COUNTDOWN_RUMBLE =
      new RobotUtil.RumbleRequest(0.65, 0);
  private static final RobotUtil.RumbleRequest ENDGAME_COUNTDOWN_RUMBLE =
      new RobotUtil.RumbleRequest(0.85, 0);
  private int lastPulseSecond = -1;
  private boolean isPulseActive = false;
  private double pulseStartTime = 0.0;

  private ShiftTimer() {
    timer = new Timer();
    Logger.recordOutput("Match/HubActive", false);
    Logger.recordOutput("Match/CurrentShift", "N/A");
    Logger.recordOutput("Match/ShiftTimer", 0.0);
  }

  public void start() {
    timer.restart();
    currentSegment = ShiftSegment.TRANSITION;
    allianceShiftNum = 0;
    isHubActive = true;
    Logger.recordOutput("Match/HubActive", true);
    Logger.recordOutput("Match/CurrentShift", currentSegment.toString());
    lastPulseSecond = -1;
    isPulseActive = false;
  }

  public void update() {
    switch (currentSegment) {
      case TRANSITION:
        if (timer.hasElapsed(10.0)) {
          timer.restart();
          resetCountdownState();
          String gameData = DriverStation.getGameSpecificMessage();
          char firstInactiveHub;
          if (!gameData.isEmpty()) firstInactiveHub = gameData.charAt(0);
          else firstInactiveHub = ' ';
          isHubActive =
              switch (firstInactiveHub) {
                case 'B' -> isRedAlliance();
                case 'R' -> !isRedAlliance();
                default -> true;
              };
          currentSegment = ShiftSegment.ALLIANCE;
          allianceShiftNum = 1;
          Logger.recordOutput("Match/CurrentShift", currentSegment + " " + allianceShiftNum);
          break;
        }
        timeRemaining = 10.0 - timer.get();
        break;
      case ALLIANCE:
        if (timer.hasElapsed(25.0)) {
          timer.restart();
          resetCountdownState();
          allianceShiftNum++;
          isHubActive = !isHubActive;
          if (allianceShiftNum > 4) {
            currentSegment = ShiftSegment.ENDGAME;
            isHubActive = true;
            Logger.recordOutput("Match/CurrentShift", currentSegment.toString());
          } else
            Logger.recordOutput(
                "Match/CurrentShift", currentSegment.toString() + " " + allianceShiftNum);
          break;
        }
        timeRemaining = 25.0 - timer.get();
        break;
      case ENDGAME:
        timeRemaining = 30.0 - timer.get();
        if (timer.hasElapsed(30.0)) {
          end();
        }
        break;
    }
    updateRumble();
    Logger.recordOutput("Match/HubActive", isHubActive);
    Logger.recordOutput("Match/ShiftTimer", timeRemaining + 1);
  }

  public void end() {
    resetCountdownState();
    timer.stop();
    isHubActive = false;
    Logger.recordOutput("Match/HubActive", false);
    Logger.recordOutput("Match/CurrentShift", "N/A");
    Logger.recordOutput("Match/ShiftTimer", 0.0);
  }

  private void resetCountdownState() {
    if (isPulseActive) {
      RobotUtil.stopDriverRumble(COUNTDOWN_RUMBLE);
      RobotUtil.stopDriverRumble(ENDGAME_COUNTDOWN_RUMBLE);
      isPulseActive = false;
    }
    lastPulseSecond = -1;
  }

  private void updateRumble() {
    if (!timer.isRunning()) return;

    if (timeRemaining <= 5.0) {
      int currentSecond = (int) Math.ceil(timeRemaining);
      if (currentSecond != lastPulseSecond && !isPulseActive) {
        RobotUtil.RumbleRequest rumble =
            currentSegment == ShiftSegment.ENDGAME ? ENDGAME_COUNTDOWN_RUMBLE : COUNTDOWN_RUMBLE;
        RobotUtil.requestDriverRumble(rumble);
        isPulseActive = true;
        lastPulseSecond = currentSecond;
        pulseStartTime = timer.get();
      }
    }

    if (isPulseActive && timer.get() - pulseStartTime >= 0.25) {
      RobotUtil.stopDriverRumble(COUNTDOWN_RUMBLE);
      RobotUtil.stopDriverRumble(ENDGAME_COUNTDOWN_RUMBLE);
      isPulseActive = false;
    }
  }
}
