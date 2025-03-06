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

  /** DO NOTHING */
  public static Command doNothing() {
    return new InstantCommand();
  }

  /** SAMPLE */
  public static Command sample(DriveSubsystem driveSubsystem) {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            driveSubsystem::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            driveSubsystem::setModuleStates,
            driveSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end
    return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, true));
  }

  /** FORWARD */
  public static Command driveForward(DriveSubsystem driveSubsystem) {
    // Create a trajectory configuration
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);

    // Define a simple trajectory: start at (0,0), drive forward 3 meters.
    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    // Create a theta controller
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Create the SwerveControllerCommand
    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            driveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            driveSubsystem::setModuleStates,
            driveSubsystem);

    // Reset odometry to the starting pose and return the auto command
    driveSubsystem.resetOdometry(trajectory.getInitialPose());
    return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, true));
  }

  /** FULL ROUTINES */
  public static Command routine1(DriveSubsystem driveSubsystem) {
    return new SequentialCommandGroup(
        driveForward(driveSubsystem),
        new WaitCommand(1.0),
        sample(driveSubsystem),
        new WaitCommand(1.0),
        new InstantCommand(() -> System.out.println("Auto ROUTINE1 complete!")));
  }
}
