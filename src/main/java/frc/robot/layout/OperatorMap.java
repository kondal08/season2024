package frc.robot.layout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Feeder.FeederDirection;
import frc.robot.Config;
import frc.robot.RobotMap.Coordinates;
import frc.robot.RobotMap.PivotMap;
import frc.robot.RobotMap.ShooterMap;
import frc.robot.Commands.IntakeUntilLoadedCommand;
import frc.robot.util.FlywheelLookupTable;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  abstract JoystickButton getIntakeButton();

  abstract JoystickButton getOuttakeButton();

  abstract JoystickButton getShootSpeakerButton();

  abstract JoystickButton getShootAmpButton();

  abstract JoystickButton getAmpAlignButton();

  abstract JoystickButton getClimbSequenceButton();

  abstract double getManualPivotAxis();

  abstract double getManualClimberAxis();

  abstract JoystickButton getArcButton();

  abstract JoystickButton getTrapButton();

  abstract JoystickButton getStageAlignButton();

  abstract JoystickButton getManualShootButton();

  abstract JoystickButton getEjectButton();

  abstract JoystickButton getAmplifyButton();

  abstract JoystickButton getCoopButton();

  abstract JoystickButton getLEDPatternOneButton();

  abstract JoystickButton getLEDPatternTwoButton();

  abstract JoystickButton getLEDPatternThreeButton();

  abstract JoystickButton getLEDPatternFourButton();

  abstract JoystickButton getLEDPatternFiveButton();

  abstract JoystickButton getLEDPatternOffButton();

  abstract Trigger getPivotRaiseButton();

  abstract Trigger getPivotLowerButton();

  abstract double getLEDAxis1();

  abstract double getLEDAxis2();

  abstract Trigger getClimberRaiseButton();

  abstract Trigger getClimberLowerButton();

  private void registerIntake() {
    if (Config.Subsystems.Intake.INTAKE_ENABLED) {
      Intake intake = Intake.getInstance();
    // getOuttakeButton().onTrue(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeDirection.REVERSE), intake));
     
      

    }
  }

  private void registerShooter() {
    if (Config.Subsystems.SHOOTER_ENABLED) {
    }
  }

  private void registerFeeder() {
    if(Config.Subsystems.FEEDER_ENABLED) {
      Feeder feeder = Feeder.getInstance();
      getShootSpeakerButton().whileTrue(new InstantCommand(() -> feeder.setFeederState(FeederDirection.FORWARD)));
      getShootAmpButton().whileTrue(new InstantCommand(() -> feeder.setFeederState(FeederDirection.FORWARD_SLOW)));
      getTrapButton().whileTrue(new InstantCommand(() -> feeder.setFeederState(FeederDirection.FORWARD)));
      getEjectButton().whileTrue(new InstantCommand(() -> feeder.setFeederState(FeederDirection.REVERSE)));
      getShootSpeakerButton().onFalse(new InstantCommand(() -> feeder.setFeederState(FeederDirection.STOPPED)));
      getShootAmpButton().onFalse(new InstantCommand(() -> feeder.setFeederState(FeederDirection.STOPPED)));
      getTrapButton().onFalse(new InstantCommand(() -> feeder.setFeederState(FeederDirection.STOPPED)));
      getEjectButton().onFalse(new InstantCommand(() -> feeder.setFeederState(FeederDirection.STOPPED)));
     

    }
  }

  private void registerClimber() {
    if (Config.Subsystems.CLIMBER_ENABLED) {
      Climber climber = Climber.getInstance();
      climber.setDefaultCommand(climber.run(() -> getManualClimberAxis()));
    }
  }

  private void registerComplexCommands(){
    if (Config.Subsystems.SHOOTER_ENABLED && Config.Subsystems.Intake.INTAKE_ENABLED
        && Config.Subsystems.DRIVETRAIN_ENABLED) {
        Shooter shooter = Shooter.getInstance();
         Pivot pivot = Pivot.getInstance();
      FlywheelLookupTable lookupTable = FlywheelLookupTable.getInstance();
      Feeder feeder = Feeder.getInstance();
      // Pose2d target = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? Coordinates.BLUE_SPEAKER : Coordinates.RED_SPEAKER;
       Pose2d target = (Config.IS_ALLIANCE_BLUE) ? Coordinates.BLUE_SPEAKER : Coordinates.RED_SPEAKER;
      PoseEstimator poseEstimator = PoseEstimator.getInstance();
      Intake intake = Intake.getInstance();
      // getShootSpeakerButton().onTrue(new ShootSequenceCommand());
      // getIntakeButton().onTrue(new RunCommand(() -> intake.setIntakeState(IntakeDirection.FORWARD), intake));
      //   getIntakeButton().onTrue(new RunCommand(() -> intake.setIntakeState(IntakeDirection.STOPPED), intake));
      getIntakeButton().onTrue(new IntakeUntilLoadedCommand());
      getArcButton().whileTrue((pivot.updatePosition(() -> lookupTable
              .get(poseEstimator.getDistanceToPose(Shooter.getInstance().findIdealTarget(() -> Drivetrain.getInstance().getPose(),
                      () -> Drivetrain.getInstance().getFieldRelativeSpeed(),
                      () -> Drivetrain.getInstance().getFieldRelativeAccel()).getTranslation())).getAngleSetpoint()).alongWith(
              shooter.setFlywheelVelocityCommand(() -> lookupTable.get(
                      poseEstimator.getDistanceToPose(Shooter.getInstance().findIdealTarget(() -> Drivetrain.getInstance().getPose(),
                              () -> Drivetrain.getInstance().getFieldRelativeSpeed(),
                              () -> Drivetrain.getInstance().getFieldRelativeAccel()).getTranslation())).getRPM()))));
      getArcButton().onFalse(shooter.setFlywheelVelocityCommand(() -> 0.0).alongWith(pivot.updatePosition(() -> -1.0)));

      getAmpAlignButton().onTrue(
        pivot.updatePosition(() -> PivotMap.PIVOT_AMP_ANGLE).alongWith(
        shooter.setFlywheelVelocityCommand(() -> ShooterMap.AMP_SPEED)));
      getAmpAlignButton().onFalse(
        shooter.setFlywheelVelocityCommand(() -> 0.0).alongWith(
        pivot.updatePosition(() -> -1.0)
        ));
      getStageAlignButton().onTrue(
        pivot.updatePosition(() -> PivotMap.PIVOT_TRAP_ANGLE).alongWith(
        shooter.setFlywheelVelocityCommand(() -> ShooterMap.TRAP_SPEED)));
      getStageAlignButton().onFalse(
        shooter.setFlywheelVelocityCommand(() -> 0.0).alongWith(
        pivot.updatePosition(() -> -1.0)
        ));

      getClimbSequenceButton().whileTrue(
          pivot.updatePosition(() -> PivotMap.PIVOT_AMP_ANGLE).andThen(
          Climber.getInstance().run(() -> 0.3))); 
      getClimbSequenceButton().onFalse(Climber.getInstance().run(() -> -0.3));

      getPivotRaiseButton().onTrue(pivot.updatePosition(() -> (PivotMap.PIVOT_AMP_ANGLE +40.0)));
      getPivotLowerButton().onTrue(pivot.updatePosition(() -> -1.0));
      getClimberRaiseButton().whileTrue(Climber.getInstance().run(() -> 0.3));
      getClimberRaiseButton().onFalse(Climber.getInstance().run(() -> -0.0));
       getClimberLowerButton().whileTrue(Climber.getInstance().run(() -> -0.3));
      getClimberLowerButton().onFalse(Climber.getInstance().run(() -> -0.0));
      // getEjectButton().whileTrue(new InstantCommand(() -> feeder.setFeederState(FeederDirection.REVERSE)).alongWith(intake.se));
    }
    
  }


  private void registerLEDs() {
    if (Config.Subsystems.LEDS_ENABLED && Config.Subsystems.FEEDER_ENABLED) {
      AddressableLEDLights lights = AddressableLEDLights.getInstance();
      Feeder feeder = Feeder.getInstance();

      getAmplifyButton().onTrue(lights.toggleAmplifyState(feeder::isNoteLoaded));
      getAmplifyButton().onTrue(lights.toggleAmplifyState(feeder::isNoteLoaded));
      // little paranoid about how suppliers work so im not gonna offload the usestate parameter 
      new Trigger(() -> !getAmplifyButton().getAsBoolean() && !getCoopButton().getAsBoolean()).whileTrue(lights.useState(lights::getState));
    }
  }

  public void registerSubsystems() {
    Intake.getInstance();
    Shooter.getInstance();
    Feeder.getInstance();
    Climber.getInstance();
  }

  @Override
  public void registerCommands() {
    registerIntake();
    registerFeeder();
    registerClimber();
    registerShooter();
    //registerLEDs();
    registerComplexCommands();

    // registerSubsystems();
  }
}
