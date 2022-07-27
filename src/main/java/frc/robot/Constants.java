// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

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

    public static final class DriveTrainConstants {
        public static final int leftFrontCANID = 1;
        public static final int leftBackCANID = 2;
        public static final int rightFrontCANID = 3;
        public static final int rightBackCANID = 4;

        public static final double ksVolts = 0.20322;
        public static final double kvVoltSecondsPerMeter = 3.2976;
        public static final double kaVoltSecondsSquaredPerMeter = 0.67542;
        public static final double kPDriveVel = 4.569;

        // public static final double ksVolts = 0.1219;
        // public static final double kvVoltSecondsPerMeter = 3.343;
        // public static final double kaVoltSecondsSquaredPerMeter = 1.0356;
        // public static final double kPDriveVel = 2.2662;

        public static final double kTrackWidthMeters = Units.inchesToMeters(23);
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackWidthMeters);

     

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kGearRatio = 12.6;
        public static final double kWheelRadiusInches = 3;

        public static final double kLinearDistanceConversionFactor = (Units
                .inchesToMeters(1 / (kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches)) * 10));

    }

}
