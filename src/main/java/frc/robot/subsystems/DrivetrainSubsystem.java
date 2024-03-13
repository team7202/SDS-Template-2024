// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveTrain.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DriveTrain.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DriveTrain.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DriveTrain.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DriveTrain.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DriveTrain.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DriveTrain.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DriveTrain.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DriveTrain.DRIVETRAIN_PIGEON_ID;
import static frc.robot.Constants.DriveTrain.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DriveTrain.DRIVETRAIN_WHEELBASE_METERS;
import static frc.robot.Constants.DriveTrain.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DriveTrain.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DriveTrain.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DriveTrain.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DriveTrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DriveTrain.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DriveTrain.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DriveTrain.FRONT_RIGHT_MODULE_STEER_OFFSET;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

        // public final PIDController feetPID = new PIDController(0.65, 0, 0);
        public final PIDController rotatePID = new PIDController(0.1, 0, 0);

        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = 12.0;
        // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // By default this value is setup for a Mk3 standard module using Falcon500s to
        // drive.
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight
         * line.
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                        SdsModuleConfigurations.MK4_L3.getDriveReduction() *
                        SdsModuleConfigurations.MK4_L3.getWheelDiameter() * Math.PI;
        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

        // By default we use a Pigeon for our gyroscope. But if you use another
        // gyroscope, like a NavX, you can change this.
        // The important thing about how you configure your gyroscope is that rotating
        // the robot counter-clockwise should
        // cause the angle reading to increase until it wraps back over to zero.
        public final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
        // private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX
        // connected over MXP

        // These are our modules. We initialize them in the constructor.
        public final SwerveModule m_frontLeftModule;
        public final SwerveModule m_frontRightModule;
        public final SwerveModule m_backLeftModule;
        public final SwerveModule m_backRightModule;

        // private boolean trailingSwitched = false;

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        public DrivetrainSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
                rotatePID.enableContinuousInput(-180, 180);

                tab.add(rotatePID);

                // There are 4 methods you can call to create your swerve modules.
                // The method you use depends on what motors you are using.
                //
                // Mk3SwerveModuleHelper.createFalcon500(...)
                // Your module has two Falcon 500s on it. One for steering and one for driving.
                //
                // Mk3SwerveModuleHelper.createNeo(...)
                // Your module has two NEOs on it. One for steering and one for driving.
                //
                // Mk3SwerveModuleHelper.createFalcon500Neo(...)
                // Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving
                // and the NEO is for steering.
                //
                // Mk3SwerveModuleHelper.createNeoFalcon500(...)
                // Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the
                // Falcon 500 is for steering.
                //
                // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper
                // class.

                // By default we will use Falcon 500s in standard configuration. But if you use
                // a different configuration or motors
                // you MUST change it. If you do not, your code will crash on startup.
                m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
                                // This parameter is optional, but will allow you to see the current state of
                                // the module on the dashboard.
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                // This can either be STANDARD or FAST depending on your gear configuration
                                Mk4iSwerveModuleHelper.GearRatio.L3,
                                // This is the ID of the drive motor
                                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                // This is the ID of the steer motor
                                FRONT_LEFT_MODULE_STEER_MOTOR,
                                // This is the ID of the steer encoder
                                FRONT_LEFT_MODULE_STEER_ENCODER,
                                // This is how much the steer encoder is offset from true zero (In our case,
                                // zero is facing straight forward)
                                FRONT_LEFT_MODULE_STEER_OFFSET);

                // We will do the same for the other modules
                m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L3,
                                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_ENCODER,
                                FRONT_RIGHT_MODULE_STEER_OFFSET);

                m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L3,
                                BACK_LEFT_MODULE_DRIVE_MOTOR,
                                BACK_LEFT_MODULE_STEER_MOTOR,
                                BACK_LEFT_MODULE_STEER_ENCODER,
                                BACK_LEFT_MODULE_STEER_OFFSET);

                m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L3,
                                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                BACK_RIGHT_MODULE_STEER_MOTOR,
                                BACK_RIGHT_MODULE_STEER_ENCODER,
                                BACK_RIGHT_MODULE_STEER_OFFSET);

                this.zeroGyroscope();

                m_frontLeftModule.setDrivePosition(0);
                m_frontRightModule.setDrivePosition(0);
                m_backLeftModule.setDrivePosition(0);
                m_backRightModule.setDrivePosition(0);
        }

        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the
         * robot is currently facing to the
         * 'forwards' direction.
         */
        public void zeroGyroscope() {
                m_pigeon.setFusedHeading(0.0);
                m_pigeon.setYaw(0.0);
        }

        public Rotation2d getGyroscopeRotation() {
                return Rotation2d.fromDegrees(m_pigeon.getYaw());
                // return Rotation2d.fromDegrees(m_pigeon.getYaw());

                // if (m_navx.isMagnetometerCalibrated()) {
                // // We will only get valid fused headings if the magnetometer is calibrated
                // return Rotation2d.fromDegrees(m_navx.getFusedHeading());
                // }
                //
                // // We have to invert the angle of the NavX so that rotating the robot
                // counter-clockwise makes the angle increase.
                // return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
        }

        public Rotation2d getRotationFromDegrees(double degrees) {
                return Rotation2d.fromDegrees(degrees);
        }

        public double getYaw() {
                return m_pigeon.getYaw();
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                this.m_chassisSpeeds = chassisSpeeds;
        }

        @Override
        public void periodic() {
                SmartDashboard.putNumber("Front Left Position", m_frontLeftModule.getDrivePosition());
                SmartDashboard.putNumber("Front Right Position", m_frontRightModule.getDrivePosition());

                SmartDashboard.putNumber("Rear Left Velocity", m_backLeftModule.getDriveVelocity());
                SmartDashboard.putNumber("Rear Right Velocity", m_backRightModule.getDriveVelocity());

                SmartDashboard.putNumber("Front Left: " + m_frontLeftModule.getSteerID(),
                                m_frontLeftModule.getSteerCurrentOutput());
                SmartDashboard.putNumber("Front Right: " + m_frontRightModule.getSteerID(),
                                m_frontRightModule.getSteerCurrentOutput());
                SmartDashboard.putNumber("Back Left: " + m_backLeftModule.getSteerID(),
                                m_backLeftModule.getSteerCurrentOutput());
                SmartDashboard.putNumber("Back Right: " + m_backRightModule.getSteerID(),
                                m_backRightModule.getSteerCurrentOutput());

                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states,
                                MAX_VELOCITY_METERS_PER_SECOND);

                m_frontLeftModule.set(states[0].speedMetersPerSecond /
                                MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond /
                                MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond /
                                MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond /
                                MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());

                SmartDashboard.putNumber("FRONT LEFT DRIVE POSITION",
                                this.m_frontLeftModule.getDrivePosition());

        }
}
