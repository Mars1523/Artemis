package frc.robot.commands.autos;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveMode;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoDrive extends Command {

    private SwerveDriveSubsystem swerveSub;
    private double goalDistance;
    private double speed;
    private Translation2d startPosition;
    SlewRateLimiter slew = new SlewRateLimiter(0.7);

    public AutoDrive(SwerveDriveSubsystem swerveSub, double goalDistance, double speed) {

        addRequirements(swerveSub);
        this.swerveSub = swerveSub;
        this.goalDistance = goalDistance;
        this.speed = speed;
        // this.slew = slew;

    }

    @Override
    public void initialize() {
        startPosition = swerveSub.getPose().getTranslation();
        slew.reset(0);
    }

    @Override
    public void execute() {
        // drive forqard

        NetworkTableInstance.getDefault()
                .getEntry("/Shuffleboard/Tune/Commanded Speed")
                .setDouble(speed);
        var ss = slew.calculate(speed);
        NetworkTableInstance.getDefault()
                .getEntry("/Shuffleboard/Tune/Limited Speed")
                .setDouble(ss);
        swerveSub.drive(ss, 0, 0, DriveMode.ROBOT);
        // hird

    }

    @Override
    public void end(boolean interrupted) {
        swerveSub.drive(0, 0, 0, DriveMode.ROBOT);
    }

    @Override
    public boolean isFinished() {

        double dist = swerveSub.getPose().getTranslation().getDistance(startPosition);
        System.out.println("distance: " + dist);
        if (goalDistance < dist) {
            return true;

        } else {
            return false;
        }
        // i finally get it now
        // goal distance is how far we want the robot to go
        // dist is how far we currently are away from the starting pos
        // if how far away we currently are is greater than how far away we want to be
        // the auto is finished
        // :DDDD
    }
}
