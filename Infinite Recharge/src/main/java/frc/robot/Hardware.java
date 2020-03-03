package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Util.doubleSolenoid;
import com.kauailabs.navx.frc.AHRS;

public class Hardware {
    // Drivetrain Motors
    public static final CANSparkMax rightMasterMotor = 
        new CANSparkMax(Constants.RIGHT_MASTER_MOTOR, MotorType.kBrushless);
    public static final CANSparkMax rightSlaveMotor = 
        new CANSparkMax(Constants.RIGHT_SLAVE_MOTOR, MotorType.kBrushless);
    public static final CANSparkMax leftMasterMotor = 
        new CANSparkMax(Constants.LEFT_MASTER_MOTOR, MotorType.kBrushless);
    public static final CANSparkMax leftSlaveMotor = 
        new CANSparkMax(Constants.LEFT_SLAVE_MOTOR, MotorType.kBrushless);

    // Cannon Motors
    public static final WPI_TalonSRX shooterMasterMotor = 
        new WPI_TalonSRX(Constants.SHOOTER_MASTER_MOTOR);
    public static final WPI_TalonSRX shooterSlaveMotor = 
        new WPI_TalonSRX(Constants.SHOOTER_SLAVE_MOTOR);
    
    public static final WPI_TalonSRX pivotMasterMotor = 
        new WPI_TalonSRX(Constants.PIVOT_MASTER_MOTOR);
    public static final WPI_VictorSPX pivotSlaveMotor = 
        new WPI_VictorSPX(Constants.PIVOT_SLAVE_MOTOR);
    
    public static final WPI_VictorSPX colorWheelMotor = 
        new WPI_VictorSPX(Constants.COLOR_SENSOR_MOTOR);
    
    public static final WPI_VictorSPX polycordMasterMotor = 
        new WPI_VictorSPX(Constants.POLYCORD_MASTER_MOTOR);
    public static final WPI_VictorSPX polycordSlaveMotor = 
        new WPI_VictorSPX(Constants.POLYCORD_SLAVE_MOTOR);
    
    // Climber Motors
    public static final WPI_TalonSRX wenchMasterMotor = 
        new WPI_TalonSRX(Constants.WENCH_MASTER_MOTOR);
    public static final WPI_VictorSPX wenchSlaveMotor = 
        new WPI_VictorSPX(Constants.WENCH_SLAVE_MOTOR);

    // Intake Motor
    public static final WPI_VictorSPX intakeMotor = 
        new WPI_VictorSPX(Constants.INTAKE_MOTOR);

    // Joystick
    public static final Joystick driverPad = 
        new Joystick(Constants.DRIVE_USB_PORT);
    
    // Shifter
    public static final doubleSolenoid shifter = 
        new doubleSolenoid(Constants.SHIFTER_OPEN, Constants.SHIFTER_CLOSE);
    public static final doubleSolenoid shifter2 = 
        new doubleSolenoid(Constants.SHIFTER2_OPEN, Constants.SHIFTER2_CLOSE);
    public static final doubleSolenoid shifter3 =
        new doubleSolenoid(Constants.SHIFTER3_OPEN, Constants.SHIFTER3_CLOSE);
        
    // Gryo
    public static final AHRS MXP = new AHRS(Constants.MXP_PORT);

    // Limit Switch
    public static final DigitalInput limiter = new DigitalInput(Constants.LIMITER);
}