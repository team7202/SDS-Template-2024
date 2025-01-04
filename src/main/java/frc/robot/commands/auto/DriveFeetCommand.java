// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.XboxController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveFeetCommand extends PIDCommand {

  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final double target;

  /** Creates a new Test. */
  public DriveFeetCommand(DrivetrainSubsystem driveTrain, double feet, boolean sideways) {
    super(
        // The controller that the command will use
        new PIDController(2.5, 0, 0),
        // This should return the measurement
        () -> feet > 0 ? Math.abs(driveTrain.m_frontRightModule.getDrivePosition())
            : -Math.abs(driveTrain.m_frontRightModule.getDrivePosition()),
        // This should return the setpoint (can also be a constant)
        () -> feet,
        // This uses the output
        output -> {
          driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
              !sideways
                  ? -MathUtil.clamp(output, -1, 1) / DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                      * DrivetrainSubsystem.MAX_VOLTAGE
                  : 0,
              sideways
                  ? MathUtil.clamp(output, -1, 1) / DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                      * DrivetrainSubsystem.MAX_VOLTAGE
                  : 0.0,
              0.0,
              driveTrain.getGyroscopeRotation()));
          SmartDashboard.putNumber("FRONT RIGHT DRIVE POSITION",
              driveTrain.m_frontRightModule.getDrivePosition());
          SmartDashboard.putNumber("FRONT RIGHT DRIVE TARGET", feet);
          // Use the output here
        });
    addRequirements(driveTrain);
    this.m_drivetrainSubsystem = driveTrain;
    this.target = feet;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize() {
    System.out.println("DRIVE FEET CALLED: " + target);
    SmartDashboard.putBoolean("DRIVE FEET ENDED", false);
    m_drivetrainSubsystem.m_frontRightModule.setDrivePosition(0);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("DRIVE FEET ENDED", true);
    System.out.println("DRIVE FEET ENDED: " + m_drivetrainSubsystem.m_frontRightModule.getDrivePosition());
    
    m_drivetrainSubsystem
        .drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, m_drivetrainSubsystem.getGyroscopeRotation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.m_controller.atSetpoint();
  }
}
