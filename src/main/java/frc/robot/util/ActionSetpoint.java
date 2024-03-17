package frc.robot.util;

public class ActionSetpoint {
  private double rpm, angleSetpoint, shotTime, feedTime;

  ActionSetpoint(double rpm, double aS, double sT, double fT) {
    this.rpm = rpm;
    this.angleSetpoint = aS;
    this.shotTime = sT;
    this.feedTime = fT;
  }

  public double getRPM() {
    return rpm;
  }

  public double getAngleSetpoint() {
    return angleSetpoint;
  }

  public double getShotTime() {
    return shotTime;
  }

  public double getFeedTime() {
    return feedTime;
  }

}
