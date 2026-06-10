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
  }

  public void update() {
    switch (currentSegment) {
      case TRANSITION:
        if (timer.hasElapsed(10.0)) {
          timer.restart();
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
    Logger.recordOutput("Match/HubActive", isHubActive);
    Logger.recordOutput("Match/ShiftTimer", timeRemaining + 1);
  }

  public void end() {
    timer.stop();
    isHubActive = false;
    Logger.recordOutput("Match/HubActive", false);
    Logger.recordOutput("Match/CurrentShift", "N/A");
    Logger.recordOutput("Match/ShiftTimer", 0.0);
  }
}
