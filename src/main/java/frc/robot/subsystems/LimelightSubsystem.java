// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LimelightX", table.getEntry("tx").getDouble(0.0));
    SmartDashboard.putNumber("LimelightY", table.getEntry("ty").getDouble(0.0));
    SmartDashboard.putNumber("LimelightArea", table.getEntry("ta").getDouble(0.0));
    SmartDashboard.putNumber("LimelightLatency", table.getEntry("tl").getDouble(0.0));

    // This method will be called once per scheduler run
  }

  public void setPipeline(int pipeline) {
    table.getEntry("pipeline").setInteger(pipeline);
  }

  public float getX() {
    return table.getEntry("tx").getFloat(0.0f);
  }

  public float getY() {
    return table.getEntry("ty").getFloat(0.0f);
  }

  public float getArea() {
    return table.getEntry("ta").getFloat(0.0f);
  }

  public float getLatency() {
    return table.getEntry("tl").getFloat(0.0f);
  }
}
