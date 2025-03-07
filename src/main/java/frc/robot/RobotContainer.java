// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.autos.Autos;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.DriveSubsystem;

// Adding imports trying to work on camera:)

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();

  // Controllers
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_attachmentController =
      new CommandXboxController(OIConstants.kAttachmentControllerPort);

  // Shuffleboard
  public final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  public final Timer teleopTimer = new Timer();
  private ShuffleboardTab board;
  private GenericEntry[] boardEntries;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
    setupShuffleboard();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));
    m_coralSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              double elevatorSpeed = m_attachmentController.getLeftY();
              double armSpeed = m_attachmentController.getRightY();
              m_coralSubsystem.runElevatorManual(elevatorSpeed);
              m_coralSubsystem.runArmManual(armSpeed);
            },
            m_coralSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
     *                          DRIVER CONTROLLER MAPPINGS
     */
    // left bumper --> set X position
    m_driverController.leftTrigger().onTrue(m_robotDrive.setXCommand());

    // right bumper --> zero gyro heading
    m_driverController.rightTrigger().onTrue(m_robotDrive.zeroHeadingCommand());

    /*
     *                          ATTACHMENT CONTROLLER MAPPINGS
     */
    // elevator
    // B Button -> Elevator/Arm to human player position
    m_attachmentController.b().onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.kFeederStation));

    // A Button -> Elevator/Arm to level 2 position
    m_attachmentController.a().onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.kLevel2));

    // X Button -> Elevator/Arm to level 3 position
    m_attachmentController.x().onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.kLevel3));

    // Y Button -> Elevator/Arm to level 4 position
    m_attachmentController.y().onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.kLevel4));

    // coral
    // Left Bumper -> Run tube intake
    m_attachmentController.leftTrigger().onTrue(m_coralSubsystem.runIntakeCommand());

    // Right Bumper -> Run tube intake in reverse
    m_attachmentController.rightTrigger().onTrue(m_coralSubsystem.reverseIntakeCommand());
  }

  private void setupShuffleboard() {
    // Chooser
    m_autoChooser.setDefaultOption("Do Nothing", Autos.doNothing());
    m_autoChooser.addOption("Sample", Autos.sample(m_robotDrive));
    m_autoChooser.addOption("Routine #1", Autos.routine1(m_robotDrive));
    SmartDashboard.putData("Auto Mode", m_autoChooser);

    // Camera
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(640, 480);

    // Shuffleboard Tab
    board = Shuffleboard.getTab("DASHBOARD");
    boardEntries = new GenericEntry[4];

    board.add("Front Camera", CameraServer.getServer().getSource());
    boardEntries[0] = board.add("Battery Voltage", RobotController.getBatteryVoltage()).getEntry();
    boardEntries[1] = board.add("Gyro Angle", m_robotDrive.getHeading().getDegrees()).getEntry();
    boardEntries[2] = board.add("Speed", m_robotDrive.getChassisSpeed()).getEntry();
    boardEntries[3] = board.add("System Timer", Timer.getFPGATimestamp()).getEntry();
  }

  public void updateShuffleboard() {
    boardEntries[0].setDouble(RobotController.getBatteryVoltage());
    boardEntries[1].setDouble(m_robotDrive.getHeading().getDegrees());
    boardEntries[2].setDouble(m_robotDrive.getChassisSpeed());
    boardEntries[3].setDouble(teleopTimer.get());
  }

  public double getSimulationTotalCurrentDraw() {
    // for each subsystem with simulation
    return m_coralSubsystem.getSimulationCurrentDraw();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
