// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveRotateCommand extends Command {

  private final DrivetrainSubsystem m_driveTrain;
  private final Rotation2d m_angle;

  private boolean finished = false;

  /** Creates a new DriveRotateCommand. */
  public DriveRotateCommand(DrivetrainSubsystem driveTrain, Rotation2d angle) {
    addRequirements(driveTrain);
    this.m_driveTrain = driveTrain;
    this.m_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentDegrees = m_driveTrain.getYaw();
    double speed = -MathUtil.clamp(m_driveTrain.rotatePID.calculate(m_driveTrain.getGyroscopeRotation().getDegrees(), m_angle.getDegrees()), -1.5,1.5);

    m_driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, speed / DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * DrivetrainSubsystem.MAX_VOLTAGE, m_driveTrain.getGyroscopeRotation()));
    SmartDashboard.putNumber("AUTON ROTATE SPEED", speed);
    SmartDashboard.putNumber("YAW", m_driveTrain.m_pigeon.getYaw());
    this.finished = Math.round(m_angle.getDegrees()) == Math.round(currentDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, m_driveTrain.getGyroscopeRotation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.finished;
  }
}
