// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class DriveTrain {
        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.51; // FIXME Measure and set trackwidth
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.679;

        public static final int DRIVETRAIN_PIGEON_ID = 62;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 13;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 12;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 24;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(278.78);
                                                                                           
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 10;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 11;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 27;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(219.8);
                                                                                            
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 2;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 25;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(215.8);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 9;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 26;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(239.58);

        public static final double Y_LIMIT = 0.8;
        public static final double X_LIMIT = 0.8;
        public static final double TWIST_LIMIT = 0.75;
    }

    public static class Arms {
        public static final int ARMS_LEFT_MOTOR = 18;
        public static final int ARMS_RIGHT_MOTOR = 21;

        // public static final double kP = 0.05;
        public static final double kP = 0.025;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double ARMS_PICKUP_POS = -15;
        public static final double ARMS_PICKUP_LOW_SPEED = -0.1;
        public static final double ARMS_PICKUP_HIGH_SPEED = 0.1;

        public static final double ARMS_ZERO_POS = 0;
        public static final double ARMS_ZERO_LOW_SPEED = -0.15;
        public static final double ARMS_ZERO_HIGH_SPEED = 0.25;

        public static final double ARMS_DRIVING_POS = -4.6;
        public static final double ARMS_DRIVING_LOW_SPEED = -0.15;
        public static final double ARMS_DRIVING_HIGH_SPEED = 0.15;

        public static final double ARMS_LOW_POS = -8.75;
        public static final double ARMS_LOW_LOW_SPEED = -0.25;
        public static final double ARMS_LOW_HIGH_SPEED = 0.25;

        public static final double ARMS_MID_POS = -8.75;
        public static final double ARMS_MID_LOW_SPEED = -0.15;
        public static final double ARMS_MID_HIGH_SPEED = 0.15;

        public static final double ARMS_AMP_POS = 4;
        public static final double ARMS_AMP_LOW_SPEED = -0.15;
        public static final double ARMS_AMP_HIGH_SPEED = 0.15;
    }

    public static class IntakeShooter {
        public static final int LEFT_SHOOTER_MOTOR = 32;
        public static final int RIGHT_SHOOTER_MOTOR = 62;
        public static final int INTAKE_MOTOR = 61;

        public static final double CLOSED_LOOP_RAMP_RATE = 2.5;

        public static final double INTAKE_SPEED = 0.6;
        public static final double SPEW_INTAKE_SPEED = -0.6;
    }

    public static class Climber {
        public static final int CLIMBER_MOTOR = 50;
        public static final double CLIMBER_DOWN_SPEED = -0.5;
        public static final double CLIMBER_UP_SPEED = 0.5;
    }

    public static class OI {
        public static final int DRIVER_STICK = 0;
    }

    public static class ButtonMap {
        public static final int ARMS_PICKUP_POV = 0;
        public static final int ARMS_ZERO_POV = 180;
        public static final int SHOOT_BUTTON = 1;
        public static final int INTAKE_BUTTON = 2;
        public static final int SLOW_SHOOT_BUTTON = 3;
        public static final int ARMS_LOW_BUTTON = 5;
        public static final int ARMS_MID_BUTTON = 6;
        public static final int ARMS_AMP_BUTTON = 7;
        public static final int CLIMBER_DOWN_BUTTON = 12;
        public static final int CLIMBER_UP_BUTTON = 13;
        public static final int SPEW_INTAKE_BUTTON = 14;
        public static final int DRIVING_ARM_BUTTON = 16;
    }
}
