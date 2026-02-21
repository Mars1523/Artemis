package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import java.io.IOException;
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

    // yagsl controller
    private SwerveDrive swerveDrive;

    // field angle at which joystick considers forward
    private Rotation2d joystickForwardAngle = Rotation2d.kZero;

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
    }

    /**
     * Drives based on saved state of preferred joystick forward angle
     *
     * @param xPercent
     * @param yPercent
     * @param rotPercent
     */
    public void driveJoystick(double xPercent, double yPercent, double rotPercent) {
        // note that x is robot-forward, y is robot-sideways
        var xSpeed = xRateLimiter.calculate(xPercent) * Constants.SwerveDriveConstants.kMaxVelocityMetersPerSecond;
        var ySpeed = yRateLimiter.calculate(yPercent) * Constants.SwerveDriveConstants.kMaxVelocityMetersPerSecond;
        var rotationSpeed = rotRateLimiter.calculate(rotPercent)
                * Constants.SwerveDriveConstants.kMaxAngularVelocityRadiansPerSecond;

        Rotation2d fieldHeading = swerveDrive.getOdometryHeading().minus(joystickForwardAngle);
        ChassisSpeeds chassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, fieldHeading);

        // `isOpenLoop=false` makes the drive motors set velocity using motor pid loops
        swerveDrive.drive(chassisSpeeds, false, new Translation2d());
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

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOmetry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    private ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    @Override
    public void periodic(){
        swerveDrive.updateOdometry();
    }
    public void acceptVisionData(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
        swerveDrive.addVisionMeasurement(pose, timestamp, estimationStdDevs);
    }
    //feed photonvision data to the odometry of Swerve Drive class YAGSL


    /**
     * resets the angle at which the joystick considers forward, based on the robot's current pose
     */
    public Command resetJoystickForwardAngle() {
        return run(() -> joystickForwardAngle = swerveDrive.getOdometryHeading());
    }
}
