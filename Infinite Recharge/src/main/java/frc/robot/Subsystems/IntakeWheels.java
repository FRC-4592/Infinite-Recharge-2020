package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Lib.SubsystemFramework;
import frc.robot.Util.doubleSolenoid;

public class IntakeWheels implements SubsystemFramework{
    private WPI_TalonSRX intakeMotor;
    private doubleSolenoid shifter2;

    public static IntakeWheelsStates state = IntakeWheelsStates.Off;

    public IntakeWheels(WPI_TalonSRX intakeMotor, doubleSolenoid shifter2) {
        this.intakeMotor = intakeMotor;
        this.shifter2 = shifter2;
    }

    public enum IntakeWheelsStates {
        Off, Intake, ReverseIntake, IntakePosition;
    }
        
    public boolean Intake() {
        return Hardware.driverPad.getRawButton(Constants.INTAKE);
    }
    public boolean Reverse() {
        return Hardware.driverPad.getRawButton(Constants.REVERSE);
    }
    public boolean IntakePosition() {
        return Hardware.driverPad.getRawButton(Constants.INTAKEPOSITION);
    }
    public void update() {
        IntakeWheelsStates newState = state;

        switch (state) {
            case Off:
                shifter2.close();
                intakeMotor.set(0);
                SmartDashboard.putString("Intake", "Off");
                
                if(Intake()) {
                    newState = IntakeWheelsStates.Intake;
                }
                else if(Reverse()) {
                    newState = IntakeWheelsStates.ReverseIntake;
                }
                else if(IntakePosition()) {
                    newState = IntakeWheelsStates.IntakePosition;
                }
                break;
            case Intake:
                shifter2.open();
                intakeMotor.set(1);
                SmartDashboard.putString("Intake", "Intake");
                
                if(Reverse()) {
                    newState = IntakeWheelsStates.ReverseIntake;
                }
                else if(IntakePosition()) {
                    newState = IntakeWheelsStates.IntakePosition;
                }
                else if(!Intake()) {
                    newState = IntakeWheelsStates.Off; 
                }
                break;
            case ReverseIntake:
                shifter2.open();
                intakeMotor.set(-1);
                SmartDashboard.putString("Intake", "Reverse");

                if(Intake()) {
                    newState = IntakeWheelsStates.Intake;
                }
                else if(IntakePosition()) {
                    newState = IntakeWheelsStates.IntakePosition;
                }
                else if(!Reverse()) {
                    newState = IntakeWheelsStates.Off;
                }
                break;
            case IntakePosition:
                shifter2.open();
                SmartDashboard.putString("Intake", "IntakePosition");

                if(Intake()) {
                    newState = IntakeWheelsStates.Intake;
                }
                else if(Reverse()) {
                    newState = IntakeWheelsStates.ReverseIntake;
                }
                else if(!IntakePosition()) {
                    newState = IntakeWheelsStates.Off;
                }
            default:
                newState = IntakeWheelsStates.Off;
                break;
        }
            if (newState != state) {
                state = newState;
            }
            outputToSmartDashboard();

    }
   
    public void outputToSmartDashboard() {
        
    }
    
    public void setupSensors() {
        intakeMotor.configNominalOutputForward(0, 10);
        intakeMotor.configPeakOutputForward(1, 10);
        intakeMotor.configNominalOutputReverse(0, 10);
        intakeMotor.configPeakOutputReverse(-1, 10);
    }
}