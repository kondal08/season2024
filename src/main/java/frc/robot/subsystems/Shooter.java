package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.RobotMap.Coordinates;
import frc.robot.RobotMap.ShooterMap;
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
     * leaderPIVOT: LEFT PIVOT
     * followerPivot: RIGHT PIVOT
     */
    private CANSparkBase top, bot;
    private SparkPIDController topPID, botPID;

    private double leadVel, followVel;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    private GenericEntry shooterAngleEntry = tab.add("Shooter Angle", 0).getEntry();
    private GenericEntry shooterRPMEntry = tab.add("Shooter RPM", 0).getEntry();
    private GenericEntry shoot_moving = tab.add("Shoot Moving?", Config.Subsystems.SHOOT_MOVING).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    FlywheelLookupTable lookupTable = FlywheelLookupTable.getInstance();
   // Pose2d target = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue) ? Coordinates.BLUE_SPEAKER : Coordinates.RED_SPEAKER;
   Pose2d target = Config.IS_ALLIANCE_BLUE ? Coordinates.BLUE_SPEAKER : Coordinates.RED_SPEAKER;
   PoseEstimator poseEstimator = PoseEstimator.getInstance();

    private Shooter() {
        if (ShooterMap.TOP_SHOOTER != -1) {

            setName("Shooter");
            top = new CANSparkFlex(ShooterMap.TOP_SHOOTER, MotorType.kBrushless);
            top.setIdleMode(IdleMode.kCoast);
            topPID = top.getPIDController();
            topPID.setP(ShooterMap.FLYWHEEL_PID.kP);
            topPID.setI(ShooterMap.FLYWHEEL_PID.kI);
            topPID.setD(ShooterMap.FLYWHEEL_PID.kD);
            topPID.setFF(ShooterMap.FLYWHEEL_FF);

            top.setClosedLoopRampRate(ShooterMap.FLYWHEEL_RAMP_RATE);
            top.burnFlash();

            var tab = Shuffleboard.getTab("Shooter");

            tab.add(this);

        }
        if (ShooterMap.BOTTOM_SHOOTER != -1) {
            bot = new CANSparkFlex(ShooterMap.BOTTOM_SHOOTER, MotorType.kBrushless);
            bot.setIdleMode(IdleMode.kCoast);
            botPID = bot.getPIDController();
            botPID.setP(ShooterMap.FLYWHEEL_PID.kP);
            botPID.setI(ShooterMap.FLYWHEEL_PID.kI);
            botPID.setD(ShooterMap.FLYWHEEL_PID.kD);
            botPID.setFF(ShooterMap.FLYWHEEL_FF);

            bot.setClosedLoopRampRate(ShooterMap.FLYWHEEL_RAMP_RATE);
            bot.burnFlash();
        }

        //initDefaultCommand();
    }

    public double getSetpoint() {
        return top.getEncoder().getPosition();
    }

    public Command setFlywheelVelocityCommand(Supplier<Double> v) {
        return new RunCommand(
                () -> {
                    leadVel = v.get();
                    followVel = v.get();
                }, this);
    }

    public Command stopFlywheelCommand() {
        return setFlywheelVelocityCommand(()->0.0);
    }

    public boolean getFlywheelIsAtVelocity(){
        return Math.abs(top.getEncoder().getVelocity() - leadVel) < ShooterMap.FLYWHEEL_VELOCITY_TOLERANCE
            && Math.abs(bot.getEncoder().getVelocity() - followVel) < ShooterMap.FLYWHEEL_VELOCITY_TOLERANCE;
    }

    public Pose2d findIdealTarget(Supplier<Pose2d> robotPose, Supplier<FieldRelativeSpeed> robotVel,
                                  Supplier<FieldRelativeAccel> robotAccel) {
        Translation2d target = Config.IS_ALLIANCE_BLUE ? Coordinates.BLUE_SPEAKER.getTranslation() : Coordinates.RED_SPEAKER.getTranslation();
        Rotation2d speakerRot = target.getAngle();

        if (!Config.Subsystems.SHOOT_MOVING) {
            return new Pose2d(target, speakerRot);
        }

        double dist = target.getDistance(robotPose.get().getTranslation());

        //fix with new lookup table that has shot time
        double shotTime = lookupTable.get(dist).getShotTime();

        //time inside bot after shot
        double feedTime = lookupTable.get(dist).getFeedTime();

        Translation2d movingGoalLocation = new Translation2d();

        for(int i=0;i<5;i++){

            double virtualGoalX = target.getX()
                    - shotTime * (robotVel.get().vx + robotAccel.get().ax * feedTime);
            double virtualGoalY = target.getY()
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
        findIdealTarget(() -> Drivetrain.getInstance().getPose(), () -> Drivetrain.getInstance().getFieldRelativeSpeed(),
                () -> Drivetrain.getInstance().getFieldRelativeAccel());
        updateMotors();
        shooterRPMEntry.setDouble(lookupTable.get(
                poseEstimator.getDistanceToPose(target.getTranslation())).getRPM());
        FlywheelLookupTable lookupTable = FlywheelLookupTable.getInstance();
        //   Pose2d target = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? Coordinates.BLUE_SPEAKER : Coordinates.RED_SPEAKER;
        PoseEstimator poseEstimator = PoseEstimator.getInstance();
        shooterAngleEntry.setDouble(lookupTable
                .get(poseEstimator.getDistanceToPose(target.getTranslation())).getAngleSetpoint());
        Config.Subsystems.SHOOT_MOVING = shoot_moving.get().getBoolean();
    }

    // public void initDefaultCommand() {
    //     setDefaultCommand(this.setFlywheelVelocityCommand(()->lookupTable.get(
    //         poseEstimator.getDistanceToPose(target.getTranslation())).getRPM()));
    // }

    private void updateMotors() {
        if (top != null) {
            if (leadVel != 0) {
                topPID.setReference(leadVel, ControlType.kVelocity);
            } else
                top.set(0);
        }
        if (bot != null) {
            if (followVel != 0) {
                botPID.setReference(followVel, ControlType.kVelocity);
            } else
                bot.set(0);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("lead velocity", () -> leadVel, (v) -> leadVel = v);
        builder.addDoubleProperty("follow velocity", () -> followVel, (v) -> followVel = v);
        builder.addDoubleProperty("real top velo", () -> top.getEncoder().getVelocity(), (d) -> {
        });
        builder.addDoubleProperty("real bottom velo", () -> bot.getEncoder().getVelocity(), (d) -> {
        });
    }

}
