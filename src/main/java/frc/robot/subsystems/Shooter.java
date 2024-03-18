package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.RobotMap;
import frc.robot.RobotMap.Coordinates;
import frc.robot.RobotMap.ShooterMap;
import frc.robot.util.ActionSetpoint;
import frc.robot.core.util.FieldRelative.FieldRelativeAccel;
import frc.robot.core.util.FieldRelative.FieldRelativeSpeed;
import frc.robot.util.FlywheelLookupTable;

/**
 * This is the Catapul-- umm... Flywheel subsystem for the 2024 season.
 * Throughout the season, add
 * everything shooting here. <br>
 * <br>
 * Think:
 *
 * <ul>
 * <li>pitch control for adjusting the launch angle,
 * <li>LUT-based power setpoints,
 * <li>closed-loop control to regain rotational momentum quickly,
 * <li>whatever else you'd like!
 * </ul>
 */
public class Shooter extends SubsystemBase {
    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    // Motor Controllers
    /*
     * leaderFlywheel: TOP FLYWHEEL
     * followerFlywheel: BOTTOM FLYWHEEL
     */
    private Optional<CANSparkBase> top, bot;

    private double leadVel, followVel;
    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    private GenericEntry shooterAngleEntry = tab.add("Shooter Angle", 0).getEntry();
    private GenericEntry shooterRPMEntry = tab.add("Shooter RPM", 0).getEntry();
    private GenericEntry shoot_moving = tab.add("Shoot Moving?", Config.Subsystems.SHOOT_MOVING).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    private FlywheelLookupTable lookupTable = ShooterMap.SHOT_LOOKUP_TABLE;
   // Pose2d target = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue) ? Coordinates.BLUE_SPEAKER : Coordinates.RED_SPEAKER;
   private Pose2d target = Config.IS_ALLIANCE_BLUE ? Coordinates.BLUE_SPEAKER : Coordinates.RED_SPEAKER;
   private PoseEstimator poseEstimator = PoseEstimator.getInstance();

    private Shooter() {
        super();

        if (ShooterMap.TOP_SHOOTER != -1) {
            top = Optional.of(new CANSparkFlex(ShooterMap.TOP_SHOOTER, MotorType.kBrushless));

            var motor = top.get();

            motor.restoreFactoryDefaults();

            motor.setIdleMode(IdleMode.kCoast);
            var pid = motor.getPIDController();
            pid.setP(ShooterMap.FLYWHEEL_PID.kP);
            pid.setI(ShooterMap.FLYWHEEL_PID.kI);
            pid.setD(ShooterMap.FLYWHEEL_PID.kD);
            pid.setFF(ShooterMap.FLYWHEEL_FF);

            motor.setClosedLoopRampRate(ShooterMap.FLYWHEEL_RAMP_RATE);
            motor.burnFlash();
        } else {
            top = Optional.empty();
        }

        if (ShooterMap.BOTTOM_SHOOTER != -1) {
            bot = Optional.of(new CANSparkFlex(ShooterMap.BOTTOM_SHOOTER, MotorType.kBrushless));

            var motor = bot.get();

            motor.restoreFactoryDefaults();

            motor.setIdleMode(IdleMode.kCoast);
            var pid = motor.getPIDController();
            pid.setP(ShooterMap.FLYWHEEL_PID.kP);
            pid.setI(ShooterMap.FLYWHEEL_PID.kI);
            pid.setD(ShooterMap.FLYWHEEL_PID.kD);
            pid.setFF(ShooterMap.FLYWHEEL_FF);

            motor.setClosedLoopRampRate(ShooterMap.FLYWHEEL_RAMP_RATE);
            motor.burnFlash();
        } else {
            bot = Optional.empty();
        }
    }

    public Command setFlywheelVelocityCommand(Supplier<Double> v) {
        return new RunCommand(
                () -> {
                    leadVel = v.get();
                    followVel = v.get();
                }, this);
    }

    public Command stopFlywheelCommand() {
        return setFlywheelVelocityCommand(() -> 0.0);
    }

    public boolean getFlywheelIsAtVelocity() {
        if (top.isEmpty() || bot.isEmpty())
            return false;
      
        return Math.abs(top.get().getEncoder().getVelocity() - leadVel) < ShooterMap.FLYWHEEL_VELOCITY_TOLERANCE
                && Math.abs(bot.get().getEncoder().getVelocity() - followVel) < ShooterMap.FLYWHEEL_VELOCITY_TOLERANCE;
    }
  
    public Pose2d findIdealTarget(Supplier<Pose2d> robotPose, Supplier<FieldRelativeSpeed> robotVel,
                                  Supplier<FieldRelativeAccel> robotAccel) {
        Translation2d targetTranslation = target.getTranslation();
        Rotation2d speakerRot = target.getRotation();

        if (!Config.Subsystems.SHOOT_MOVING) {
            return target;
        }

        double dist = targetTranslation.getDistance(robotPose.get().getTranslation());

        //fix with new lookup table that has shot time
        double shotTime = lookupTable.get(dist).getShotTime();

        //time inside bot after shot
        double feedTime = lookupTable.get(dist).getFeedTime();

        Translation2d movingGoalLocation = new Translation2d();

        for(int i=0;i<5;i++){

            double virtualGoalX = targetTranslation.getX()
                    - shotTime * (robotVel.get().vx + robotAccel.get().ax * feedTime);
            double virtualGoalY = targetTranslation.getY()
                    - shotTime * (robotVel.get().vy + robotAccel.get().ay * feedTime);

            Translation2d testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

            dist = testGoalLocation.getDistance(robotPose.get().getTranslation());

            double newShotTime = lookupTable.get(dist).getShotTime();

            feedTime = lookupTable.get(dist).getFeedTime();

            if(Math.abs(newShotTime-shotTime) <= 0.010){
                i=4;
            }

            if(i == 4){
                movingGoalLocation = testGoalLocation;
            }
            else{
                shotTime = newShotTime;
            }

        }

        return new Pose2d(movingGoalLocation, speakerRot);

    }

    @Override
    public void periodic() {
        Pose2d target = findIdealTarget(() -> Drivetrain.getInstance().getPose(), () -> Drivetrain.getInstance().getFieldRelativeSpeed(),
                () -> Drivetrain.getInstance().getFieldRelativeAccel());
        updateMotors();
        shooterRPMEntry.setDouble(lookupTable.get(
                poseEstimator.getDistanceToPose(target.getTranslation())).getRPM());
        FlywheelLookupTable lookupTable = ShooterMap.SHOT_LOOKUP_TABLE;
        //   Pose2d target = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? Coordinates.BLUE_SPEAKER : Coordinates.RED_SPEAKER;
        PoseEstimator poseEstimator = PoseEstimator.getInstance();
        shooterAngleEntry.setDouble(lookupTable
                .get(poseEstimator.getDistanceToPose(target.getTranslation())).getAngle());
        Config.Subsystems.SHOOT_MOVING = shoot_moving.get().getBoolean();
    }

    private void updateMotors() {
        if (top.isPresent()) {
            if (leadVel != 0) {
                top.get().getPIDController().setReference(leadVel, ControlType.kVelocity);
            } else
                top.get().set(0);
        }
        if (bot.isPresent()) {
            if (followVel != 0) {
                bot.get().getPIDController().setReference(followVel, ControlType.kVelocity);
            } else
                bot.get().set(0);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("lead velocity", () -> leadVel, (v) -> leadVel = v);
        builder.addDoubleProperty("follow velocity", () -> followVel, (v) -> followVel = v);
        builder.addDoubleProperty("real top velo",
                () -> top.map(motor -> motor.getEncoder().getVelocity()).orElse(0.0),
                (d) -> {}
        );
        builder.addDoubleProperty("real bot velo",
                () -> bot.map(motor -> motor.getEncoder().getVelocity()).orElse(0.0),
                (d) -> {}
        );

        Supplier<ActionSetpoint> getSetpoint = () -> lookupTable.get(poseEstimator.getDistanceToPose(target.getTranslation()));

        builder.addDoubleProperty("lookup rpm", () -> getSetpoint.get().getRPM(), (d) -> {
        });
        builder.addDoubleProperty("lookup angle", () -> getSetpoint.get().getAngle(), (d) -> {
        });
    }

}
