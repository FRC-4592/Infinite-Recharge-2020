package frc.robot;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;

public class Constants {
    // Drivetrain Motors
    public static final int RIGHT_MASTER_MOTOR = 1;
    public static final int RIGHT_SLAVE_MOTOR = 2;
    public static final int LEFT_MASTER_MOTOR = 3;
    public static final int LEFT_SLAVE_MOTOR = 4;

    // Cannon Motors
    public static final int SHOOTER_MASTER_MOTOR = 0;
    public static final int SHOOTER_SLAVE_MOTOR = 1;
    public static final int PIVOT_MASTER_MOTOR = 2;
    public static final int PIVOT_SLAVE_MOTOR = 3;
    public static final int COLOR_SENSOR_MOTOR = 4;
    public static final int POLYCORD_MASTER_MOTOR = 5;
    public static final int POLYCORD_SLAVE_MOTOR = 6;

    // Climber Motors
    public static final int WENCH_MASTER_MOTOR = 7;
    public static final int WENCH_SLAVE_MOTOR = 8;

    // Intake Motor
    public static final int INTAKE_MOTOR = 9;

    // Joystick
    public static final int DRIVE_USB_PORT = 0;

    // Shifter
    public static final int SHIFTER_OPEN = 0;
    public static final int SHIFTER_CLOSE = 1;
    public static final int SHIFTER2_OPEN = 2;
    public static final int SHIFTER2_CLOSE = 3;
    public static final int SHIFTER3_OPEN = 4;
    public static final int SHIFTER3_CLOSE = 5;

    // Limit Switch
    public static final int LIMITER = 0;

    // Gyro
    public static final Port MXP_PORT = SPI.Port.kMXP;

    // Color Sensor Value
    public static final ColorSensorV3 colorSensor = 
        new ColorSensorV3(I2C.Port.kOnboard);

    // Drivetrain Buttons
    public static final int DRIVETRAIN_LOWGEAR = 1;
    public static final int DRIVETRAIN_HIGHGEAR = 2;

    // Intake Buttons
    public static final int INTAKE = 3; // Buttons will need to be changed later

    // Color Wheel Buttons
    public static final int RED = 4;
    public static final int BLUE = 5;
    public static final int GREEN = 6;
    public static final int YELLOW = 7;
    public static final int SPIN = 8;

    // Cannon Buttons
    public static final int STARTPOSITION = 9;
    public static final int ANGLEDPOSITION = 10;

    // Wench Buttons
    public static final int CLIMBEROPEN = 11;
    public static final int WENCHPULL = 12;

    // Cannon Shoot Trigger
    public static final int SHOOT = 2;

    // Drivetrain PID Values
    public static final double Drivetrain_Kf = 0;
    public static final double Drivetrain_Kp = 0.1; //This will change but for now
    public static final double Drivetrain_Ki = 0;
    public static final double Drivetrain_Kd = 0;
    public static final double Drivetrain_Wheel_Radius = 3;
    public static final double Drivetrain_Wheel_Circumference = Drivetrain_Wheel_Radius * 2 * Math.PI;
    public static final double Drivetrain_Gear_Ratio = 0;

    // Cannon Pivot PID Values
    public static final double CannonPivot_Kf = 0;
    public static final double CannonPivot_Kp = 0;
    public static final double CannonPivot_Ki = 0;
    public static final double CannonPivot_Kd = 0;
    public static final double Average_Ticks_Per_Degree = 0;

    // Drivetrain Characterization/Trajectory Values
    public static final double ksVolts = 0.0;
    public static final double kvVoltSecondsPerMeter = 0.0;
    public static final double kaVoltsSecondsSquaredPerMeter = 0.0;
    public static final double kPDriveVel = 0.0;
    public static final double kTrackWidthMeters = 0.0;
    public static final DifferentialDriveKinematics kDriveKinematics =
    new DifferentialDriveKinematics(kTrackWidthMeters);
    public static final int kEncoderCountsPerRevolution = 42;
    public static final double kWheelDiameterMeters = 6.0;
    public static final double kEncoderDistancePerPulse = 
    (kWheelDiameterMeters * Math.PI) / (double) kEncoderCountsPerRevolution;

    // Not extremely crucial but please find
    public static final double kMaxSpeedMetersPerSecond = 0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0;
    
    // Tune later but might work fine as is
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

}