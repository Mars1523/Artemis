package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class DefaultSwerve extends Command {

    private Joystick joy;
    private SwerveDriveSubsystem swerveSub;
    boolean slow = false;

    public DefaultSwerve(Joystick joy, SwerveDriveSubsystem swerveSub) {
        addRequirements(swerveSub);
        this.swerveSub = swerveSub;
        this.joy = joy;
    }

    private double signedPow(double a, double pow) {
        return Math.copySign(Math.pow(a, pow), a);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        // swerve stuff goes here
        // xspeed is xbox controller left joystick yspeed is also left joystick and
        // rotation is right joystick

        // adding deadbands

        var xSpeed = (MathUtil.applyDeadband(-joy.getY(), 0.1));
        var ySpeed = (MathUtil.applyDeadband(-joy.getX(), 0.1));
        var rot = (MathUtil.applyDeadband(-joy.getTwist(), 0.1));

        // by multiplying both xSpeed and ySpeed by this number,
        // we effectively apply the signedPow(2) behavior to the overall speed
        // instead of xSpeed and ySpeed independently
        var squareSpeedFactor = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);
        xSpeed *= squareSpeedFactor;
        ySpeed *= squareSpeedFactor;

        rot = signedPow(rot * .7, 3);

        if (!joy.getTrigger()) {
            xSpeed *= 0.5;
            ySpeed *= 0.5;
            rot *= 0.8;
        } else {
            rot *= 1;
            xSpeed *= 0.8;
            ySpeed *= 0.8;
        }

        if (joy.getRawButton(7)) {
            xSpeed *= 0.75;
            ySpeed *= 0.75;
            rot *= 0.25;
        }

        swerveSub.drive(xSpeed, ySpeed, rot, DriveMode.JOYSTICK);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
