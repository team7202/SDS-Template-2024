// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAmpCommand extends Command {

  private final DrivetrainSubsystem m_drivetrain;
  private final LimelightSubsystem m_limelight;

  private final PIDController yPID = new PIDController(0.06, 0, 0);
  private final PIDController xPID = new PIDController(0.06, 0, 0);

  private boolean finished = false;

  /** Creates a new AutoAmpCommand. */
  public AutoAmpCommand(DrivetrainSubsystem drivetrain, LimelightSubsystem limelight) {
    m_drivetrain = drivetrain;
    m_limelight = limelight;

    // Shuffleboard.getTab("PID").add(drivePID);

    addRequirements(drivetrain, limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setPipeline(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = MathUtil.clamp(xPID.calculate(Math.floor(m_limelight.getX()), 0), -0.25, 0.25);
    double y = MathUtil.clamp(yPID.calculate(Math.floor(m_limelight.getY()), 0), -0.25, 0.25);

    if (Math.abs(m_limelight.getX()) > 5 || Math.abs(m_limelight.getY()) > 5)
      m_drivetrain.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              y / DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * DrivetrainSubsystem.MAX_VOLTAGE,
              x / DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * DrivetrainSubsystem.MAX_VOLTAGE,
              0,
              m_drivetrain.getGyroscopeRotation()));
    else
      m_drivetrain.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              0,
              0, 0,
              m_drivetrain.getGyroscopeRotation()));
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
