//I'm trying -oscar

package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;


public final class Autos {

    // A command to make the robot do nothing (useful for testing or default)
    public static Command doNothing() {
        return new InstantCommand(); // InstantCommand does nothing immediately
    }

    // A command to drive the robot forward using a trajectory
    public static Command driveForward(DriveSubsystem driveSubsystem) {
        // Create a trajectory configuration (speed and acceleration)
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        )
        .setKinematics(DriveConstants.kDriveKinematics); // Use the robot's kinematics for proper movement

        // Generate a trajectory: start at (0,0), move forward 3 meters, end at (3,0)
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),  // Start at origin (0,0)
            new ArrayList<>(),                    // No intermediate waypoints
            new Pose2d(3, 0, new Rotation2d(0)),  // End at (3,0)
            config
        );

        // Create a command to follow the trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,                      // The trajectory to follow
            driveSubsystem::getPose,         // Get the robot's current pose
            DriveConstants.kDriveKinematics, // The robot's kinematics
            new PIDController(AutoConstants.kPXController, 0, 0),  // PID controller for X
            new PIDController(AutoConstants.kPYController, 0, 0),  // PID controller for Y
            new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),  // Theta controller for rotation
            driveSubsystem::setModuleStates, // Function to set the drive module states
            driveSubsystem                  // The drive subsystem (used to control the robot)
        );

        // Reset odometry to the start of the trajectory and execute the command
        driveSubsystem.resetOdometry(trajectory.getInitialPose());

        // Return the command to follow the trajectory, then stop the robot after completion
        return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, true));
    }

    //A trajectory that goes to the left, and moves up
    public static Command leftAndUp(DriveSubsystem driveSubsystem) {

        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxAccelerationMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(DriveConstants.kDriveKinematics)


        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), //we start at the origin
            new ArrayList<>(),
            new Pose2d(-3, 0), new Rotation2d(0)),
            new ArrayList<>(), 
            new Pose2d(0, 3), new Rotation2d(0)),
            config
        );

        // SwerveControllerCommand to follow the trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            driveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),
            driveSubsystem::setModuleStates,
            driveSubsystem
        );

        // Reset odometry to the starting pose and execute the command
        driveSubsystem.resetOdometry(trajectory.getInitialPose());

        return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, true)); // Stop after finishing
        }
    }

    // A full autonomous routine that does something more complex (drive, wait, then sample)
    public static Command routine1(DriveSubsystem driveSubsystem) {
        return new SequentialCommandGroup(
            driveForward(driveSubsystem),  // First, drive forward
            new WaitCommand(1.0),          // Wait for 1 second
            doNothing(),                   // Do nothing (just for testing or sequence)
            new WaitCommand(1.0),          // Wait again for 1 second
            new InstantCommand(() -> System.out.println("Autonomous routine complete!"))  // Print a message
        );
    }
}
