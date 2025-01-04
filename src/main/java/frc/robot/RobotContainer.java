// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Arms.ARMS_AMP_HIGH_SPEED;
import static frc.robot.Constants.Arms.ARMS_AMP_LOW_SPEED;
import static frc.robot.Constants.Arms.ARMS_AMP_POS;
import static frc.robot.Constants.Arms.ARMS_LOW_HIGH_SPEED;
import static frc.robot.Constants.Arms.ARMS_LOW_LOW_SPEED;
import static frc.robot.Constants.Arms.ARMS_LOW_POS;
import static frc.robot.Constants.Arms.ARMS_PICKUP_HIGH_SPEED;
import static frc.robot.Constants.Arms.ARMS_PICKUP_LOW_SPEED;
import static frc.robot.Constants.Arms.ARMS_PICKUP_POS;
import static frc.robot.Constants.Arms.ARMS_ZERO_HIGH_SPEED;
import static frc.robot.Constants.Arms.ARMS_ZERO_LOW_SPEED;
import static frc.robot.Constants.Arms.ARMS_ZERO_POS;
import static frc.robot.Constants.ButtonMap.ARMS_LOW_BUTTON;
import static frc.robot.Constants.ButtonMap.ARMS_AMP_BUTTON;
import static frc.robot.Constants.ButtonMap.ARMS_PICKUP_POV;
import static frc.robot.Constants.ButtonMap.ARMS_ZERO_POV;
import static frc.robot.Constants.ButtonMap.CLIMBER_DOWN_BUTTON;
import static frc.robot.Constants.ButtonMap.CLIMBER_UP_BUTTON;
import static frc.robot.Constants.ButtonMap.INTAKE_BUTTON;
import static frc.robot.Constants.ButtonMap.SHOOT_BUTTON;
import static frc.robot.Constants.ButtonMap.SPEW_INTAKE_BUTTON;
import static frc.robot.Constants.Climber.CLIMBER_DOWN_SPEED;
import static frc.robot.Constants.Climber.CLIMBER_UP_SPEED;
import static frc.robot.Constants.DriveTrain.TWIST_LIMIT;
import static frc.robot.Constants.DriveTrain.X_LIMIT;
import static frc.robot.Constants.DriveTrain.Y_LIMIT;
import static frc.robot.Constants.IntakeShooter.INTAKE_SPEED;
import static frc.robot.Constants.IntakeShooter.SPEW_INTAKE_SPEED;
import static frc.robot.Constants.OI.DRIVER_PORT;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.CenterAutonCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.MoveArmsCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SideAutonCommand;
import frc.robot.commands.auto.amp.AutoAmpSequentialCommand;
import frc.robot.subsystems.ArmsSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.RingSensorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  // @SuppressWarnings("unused")
  // private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();

  // public static final RingSensorSubsystem m_ringSensorSubsystem = new RingSensorSubsystem();
  // private final ArmsSubsystem m_armsSubsystem = new ArmsSubsystem();
  // private final IntakeShooterSubsystem m_intakeShooterSubsystem = new IntakeShooterSubsystem();
  // private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  // private final AutoAmpCommand m_autoAmpCommand = new
  // AutoAmpCommand(m_drivetrainSubsystem, m_limelightSubsystem);

  // private final AutoAmpSequentialCommand m_autoAmpCommand = new AutoAmpSequentialCommand(m_drivetrainSubsystem,
  //     m_armsSubsystem, m_limelightSubsystem);

  // private final CenterAutonCommand centerAutonCommand = new CenterAutonCommand(
  //     m_intakeShooterSubsystem, m_drivetrainSubsystem, m_armsSubsystem, m_ringSensorSubsystem);

  // private Command autoCommand = centerAutonCommand;

  // private final SideAutonCommand sideAutonCommand = new SideAutonCommand(m_intakeShooterSubsystem,
  //     m_drivetrainSubsystem, m_armsSubsystem);

  // @SuppressWarnings("unused")
  // private final AmpCommand ampCommand = new AmpCommand(m_armsSubsystem, m_intakeShooterSubsystem);

  public static final Joystick joystick = new Joystick(DRIVER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    // ShuffleboardTab dashboardTab = Shuffleboard.getTab("dashboard");

    // SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    // autoChooser.setDefaultOption("Center", centerAutonCommand);
    // autoChooser.addOption("Sides", sideAutonCommand);

    // autoChooser.onChange((command) -> this.autoCommand = command);

    // dashboardTab.add(autoChooser);

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(-joystick.getY() * Y_LIMIT) *
            DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(-joystick.getX() * X_LIMIT) *
            DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(-joystick.getTwist() * TWIST_LIMIT)
            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    // m_drivetrainSubsystem,
    // () -> -modifyAxis(-controller.getLeftY()) *
    // DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    // () -> -modifyAxis(-controller.getLeftX()) *
    // DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    // () -> -modifyAxis(-controller.getRightX() * TWIST_LIMIT)
    // * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // new JoystickButton(joystick, 4).onTrue(m_autoAmpCommand);

    // new POVButton(joystick, ARMS_PICKUP_POV)
    //     .onTrue(new MoveArmsCommand(m_armsSubsystem, ARMS_PICKUP_POS, ARMS_PICKUP_LOW_SPEED, ARMS_PICKUP_HIGH_SPEED));

     //new JoystickButton(joystick, 1)
         //.whileTrue(m_autoAmpCommand);

    // new JoystickButton(joystick, 2).onTrue(new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()));

    // new POVButton(joystick, ARMS_ZERO_POV)
    //     .onTrue(new MoveArmsCommand(m_armsSubsystem, ARMS_ZERO_POS,
    //         ARMS_ZERO_LOW_SPEED, ARMS_ZERO_HIGH_SPEED));

    // new JoystickButton(joystick, INTAKE_BUTTON)
    //     .whileTrue(new RunCommand(() -> {
    //       if (m_ringSensorSubsystem.ringDetected()) {
    //         m_intakeShooterSubsystem.setSpeed(m_intakeShooterSubsystem.shooterSpeed, 0);
    //         m_armsSubsystem.setTarget(ARMS_LOW_POS, ARMS_LOW_LOW_SPEED,
    //             ARMS_LOW_HIGH_SPEED);
    //       } else
    //         m_intakeShooterSubsystem.setSpeed(m_intakeShooterSubsystem.shooterSpeed,
    //             INTAKE_SPEED);
    //     }, m_ringSensorSubsystem, m_intakeShooterSubsystem))
    //     .onFalse(new RunCommand(() -> m_intakeShooterSubsystem.setSpeed(0, 0)));

    // new JoystickButton(joystick, SPEW_INTAKE_BUTTON)
    //     .whileTrue(new RunCommand(() -> {
    //       m_intakeShooterSubsystem.setSpeed(m_intakeShooterSubsystem.shooterSpeed,
    //           SPEW_INTAKE_SPEED);
    //     }, m_intakeShooterSubsystem))
    //     .onFalse(new RunCommand(() -> m_intakeShooterSubsystem.setSpeed(0, 0)));

    // new JoystickButton(joystick, SHOOT_BUTTON)
    //     .whileTrue(new ShootCommand(m_intakeShooterSubsystem, m_armsSubsystem))
    //     .onFalse(new RunCommand(() -> m_intakeShooterSubsystem.setSpeed(0, 0)));

    // new JoystickButton(joystick, ARMS_LOW_BUTTON)
    //     .onTrue(new MoveArmsCommand(m_armsSubsystem, ARMS_LOW_POS,
    //         ARMS_LOW_LOW_SPEED, ARMS_LOW_HIGH_SPEED));

    // new JoystickButton(joystick, ARMS_AMP_BUTTON)
    //     .onTrue(new MoveArmsCommand(m_armsSubsystem, ARMS_AMP_POS,
    //         ARMS_AMP_LOW_SPEED, ARMS_AMP_HIGH_SPEED));

    // new JoystickButton(joystick, CLIMBER_DOWN_BUTTON)
    //     .onTrue(new InstantCommand(() -> m_climberSubsystem.setSpeed(CLIMBER_DOWN_SPEED)))
    //     .onFalse(new InstantCommand(() -> m_climberSubsystem.setSpeed(0)));

    //new JoystickButton(joystick, 15).whileTrue(new InstantCommand(() -> m_armsSubsystem.setSpeed(0.5))).onFalse(new InstantCommand(() -> m_armsSubsystem.stop()));
    //new JoystickButton(joystick, 16).whileTrue(new InstantCommand(() -> m_armsSubsystem.setSpeed(-0.5))).onFalse(new InstantCommand(() -> m_armsSubsystem.stop()));

    // new JoystickButton(joystick, 9).onTrue(new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.15);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
