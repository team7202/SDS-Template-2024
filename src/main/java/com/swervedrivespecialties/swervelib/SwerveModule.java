package com.swervedrivespecialties.swervelib;

public interface SwerveModule {
    double getDriveVelocity();
    double getDrivePosition();
    void setDrivePosition(double pos);

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    double getSteerCurrentOutput();
    double getSteerID();
}
