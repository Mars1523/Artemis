package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.NTDouble;
import java.io.File;
import java.io.IOException;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * The swerve drive is configured using YAGSL - https://docs.yagsl.com/
 * This allows the swerve modules and motors to be initialized with config files
 * instead of a separate swerve module subsystem class (eg Apollo).
 * The config files are located in deploy/swerve.
 * Currently they are copied from Mariner but need to be updated for the 2026 practice bot once built.
 * yagsl docs on config files: https://docs.yagsl.com/configuring-yagsl/configuration
 * web tool for generating config files: https://yet-another-software-suite.github.io/YAGSL/config_generator/
 */
public class SwerveDriveSubsystem extends SubsystemBase {
    // caps each max translational accelerations at 2 m/s^2
    private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(2);
    // caps the max rotational acceleration at 2 rad/s^2
    private final SlewRateLimiter rotRateLimiter = new SlewRateLimiter(2);
    private boolean isShooting = false;

    // yagsl controller
    private SwerveDrive swerveDrive;

    // field angle at which joystick considers forward
    private Rotation2d joystickForwardAngle = Rotation2d.kZero;

    public NTDouble speedReductonWhenShooting = new NTDouble(.5, "Swerve/SpeedReductionWhenShooting");
    public NTDouble maxVelociyWhenShooting = new NTDouble(1.2, "Swerve/MaxVelocityWhenShooting");
    public NTDouble rotationReductionWhenShooting = new NTDouble(0.5, "Swerve/RotationReductionWhenShooting");
    public NTDouble maxRotationRateWhenShooting =
            new NTDouble(1.0, "Swerve/MaxRotationRateWhenShooting"); // radians/sec

    public SwerveDriveSubsystem() {
        // example code from yagsl: https://docs.yagsl.com/configuring-yagsl/code-setup
        File swerveConfigDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveConfigDirectory).createSwerveDrive(0);
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }

        // causes the robot to maintain its current heading if the rotation angle is small enough and the translation
        // speed is high enough
        // useful for teleop, not sure if desired for auto or pathplanning
        // yagsl docs: https://docs.yagsl.com/overview/our-features/heading-correction
        swerveDrive.setHeadingCorrection(true);

        // causes the drive wheels to move slower when facing the away from the desired direction by a factor of
        // cos(theta)
        // yagsl docs: https://docs.yagsl.com/overview/our-features/cosine-compensation
        // breaks in simulation mode
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);

        // helps reduce skew (translating incorrectly while rotating at higher speeds)
        // default value in docs is 0.1, recommended to set between -.15 and .15, can manually tune to minimize skew
        // yagsl docs: https://docs.yagsl.com/overview/our-features/angular-velocity-compensation
        // read about skew:
        // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);

        // if robot is on red team, set the forward angle to the left

        initializeAuto();
    }

    /**
     * Initializes callbacks needed for Autos
     * code copied here: https://pathplanner.dev/pplib-build-an-auto.html#configure-autobuilder
     */
    private void initializeAuto() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
        // Configure AutoBuilder las
        boolean enableFeedforward = true;
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a
                // starting pose)
                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speedsRobotRelative, moduleFeedForwards) -> {
                    if (enableFeedforward) {
                        swerveDrive.drive(
                                speedsRobotRelative,
                                swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                moduleFeedForwards.linearForces());
                    } else {
                        swerveDrive.setChassisSpeeds(speedsRobotRelative);
                    }

                    // var swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(
                    // ChassisSpeeds.discretize(speeds, .02));
                    // driveStates(swerveModuleStates);
                }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely
                        // live
                        // in your
                        // Constants class
                        new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(3, 0.0, 0.0) // Rotation PID constants
                        // Max module speed, in m/s // Drive base radius in meters. Distance from robot
                        // center to
                        // furthest module.
                        // Default path replanning config. See the API
                        // for the options here
                        ),
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
                );
        PathfindingCommand.warmupCommand().schedule();
    }

    public boolean getIsShooting() {
        return isShooting;
    }

    public void setIsShooting(boolean isShooting) {
        this.isShooting = isShooting;
    }

    /**
     * Drives based on saved state of preferred joystick forward angle
     *
     * @param xPercent
     * @param yPercent
     * @param rotPercent
     * @param driveMode
     */
    public void drive(double xPercent, double yPercent, double rotPercent, DriveMode driveMode) {
        var xSpeed = xRateLimiter.calculate(xPercent) * Constants.DriveConstants.kMaxVelocityMetersPerSecond;
        var ySpeed = yRateLimiter.calculate(yPercent) * Constants.DriveConstants.kMaxVelocityMetersPerSecond;
        double rotationSpeed =
                rotRateLimiter.calculate(rotPercent) * Constants.DriveConstants.kMaxAngularVelocityRadiansPerSecond;

        Translation2d translation = new Translation2d(xSpeed, ySpeed);
        if (isShooting) {
            translation = translation.times(speedReductonWhenShooting.get());
            if (translation.getNorm() > maxVelociyWhenShooting.get()) {
                translation = translation.times(maxVelociyWhenShooting.get() / translation.getNorm());
            }
            rotationSpeed *= rotationReductionWhenShooting.get();
            if (rotationSpeed > maxRotationRateWhenShooting.get()) {
                rotationSpeed = maxRotationRateWhenShooting.get();
            }
        }

        switch (driveMode) {
            // case FIELD:
            // swerveDrive.drive(new Translation2d(xSpeed, ySpeed), rotationSpeed, true,
            // false);
            // break;
            case ROBOT:
                swerveDrive.drive(translation, rotationSpeed, false, false);
                break;
            case JOYSTICK:
                // Optional
                // translation = SwerveMath.cubeTranslation(translation);
                if (DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red) {
                    translation = translation.rotateBy(Rotation2d.k180deg);
                }
                Rotation2d fieldHeading = swerveDrive.getOdometryHeading().minus(joystickForwardAngle);
                ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), translation.getY(), rotationSpeed, fieldHeading);
                swerveDrive.drive(chassisSpeeds, false, new Translation2d());
                break;
        }
    }

    /**
     * sets drive motor voltages to zero, while leaving turning motors in place
     */
    public void stop() {
        SwerveModuleState[] states = swerveDrive.getStates();
        for (var state : states) {
            state.speedMetersPerSecond = 0;
        }
        // `isOpenLoop=true` sets the motor voltages to zero directly, instead of setting pid loops
        swerveDrive.setModuleStates(states, true);
    }

    public Rotation2d getRotation() {
        return swerveDrive.getOdometryHeading();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getRealFieldVelocity();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
        Logger.recordOutput("Swerve/Pose2D", getPose());

        ChassisSpeeds robotVelocity = getRobotVelocity();
        double robotSpeed = Math.sqrt(robotVelocity.vxMetersPerSecond * robotVelocity.vxMetersPerSecond
                + robotVelocity.vyMetersPerSecond * robotVelocity.vyMetersPerSecond);
        Logger.recordOutput("Swerve/RobotRotationRate", robotVelocity.omegaRadiansPerSecond);
        Logger.recordOutput("Swerve/RobotVelocity", robotVelocity);
        Logger.recordOutput("Swerve/RobotSpeed", robotSpeed);
        Logger.recordOutput("Swerve/isShooting", isShooting);
    }

    public void acceptVisionData(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
        swerveDrive.addVisionMeasurement(pose, timestamp, estimationStdDevs);
    }
    // feed photonvision data to the odometry of Swerve Drive class YAGSL

    /**
     * resets the angle at which the joystick considers forward, based on the robot's current pose
     */
    public Command resetJoystickForwardAngle() {
        return run(() -> {
            Rotation2d currOdometryHeading = swerveDrive.getOdometryHeading();
            if (DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red) {
                currOdometryHeading = currOdometryHeading.rotateBy(Rotation2d.k180deg);
            }
            joystickForwardAngle = currOdometryHeading;
        });
    }

    public ChassisSpeeds getRealFieldVelocity() {
        // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
        // but not the reverse.  However, because this transform is a simple rotation, negating the
        // angle given as the robot angle reverses the direction of rotation, and the conversion is reversed.
        // ChassisSpeeds robotRelativeSpeeds = swerveDrive.kinematics.toChassisSpeeds(swerveDrive.getStates());
        // return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, swerveDrive.getOdometryHeading());
        // Might need to be this instead
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                swerveDrive.kinematics.toChassisSpeeds(swerveDrive.getStates()),
                swerveDrive.getOdometryHeading().unaryMinus());
    }
}
