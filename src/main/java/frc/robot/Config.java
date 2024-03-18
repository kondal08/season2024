package frc.robot;

import frc.robot.core.util.controllers.BoardController;
import frc.robot.core.util.controllers.ButtonMap;
import frc.robot.core.util.controllers.Xbox;

public class Config {

  public static final boolean IS_ALLIANCE_RED = true;
  public static final boolean IS_ALLIANCE_BLUE = !IS_ALLIANCE_RED;
  public static final class Subsystems {
      public static final boolean DRIVETRAIN_ENABLED = true;
      public static final boolean CLIMBER_ENABLED = false;
      public static final boolean SHOOTER_ENABLED = true;
      public static boolean SHOOT_MOVING = false;
      public static boolean VISION_ENABLED = true;
      public static final boolean PIVOT_ENABLED = true;
      public static final boolean INTAKE_ENABLED = true;
      public static final boolean FEEDER_ENABLED = true;

      public static final boolean LEDs_ENABLED = false;

      public class Controllers {
          public static final boolean DRIVER_ENABLED = true;
          public static final boolean JOYSTICK_OPERATOR_ENABLED = false;
          public static final boolean OPERATOR_ENABLED = true;
          public static final boolean BOARD_OPERATOR_ENABLED = true;

          public static ButtonMap getDriverController() {
              return new Xbox();
          }

          public static ButtonMap getOperatorController() {
              return BOARD_OPERATOR_ENABLED ? new BoardController() : new Xbox();
          }
      }
  }
}
