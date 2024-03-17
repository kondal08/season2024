package frc.robot.util;

public class ActionSetpointBuilder {
  private double rpm, angleSetpoint, shotTime, feedTime;

  public ActionSetpointBuilder(double rpm, double angleSetpoint, double shotTime, double feedTime) {
    this.rpm = rpm;
    this.angleSetpoint = angleSetpoint;
    this.shotTime = shotTime;
    this.feedTime = feedTime;
  }

  public ActionSetpoint build() {
    return new ActionSetpoint(rpm, angleSetpoint, shotTime, feedTime);
  }
}
