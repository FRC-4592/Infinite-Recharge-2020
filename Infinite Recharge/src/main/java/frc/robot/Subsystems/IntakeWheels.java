package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Lib.SubsystemFramework;
import frc.robot.Util.doubleSolenoid;

public class IntakeWheels implements SubsystemFramework{
    private WPI_VictorSPX intakeMotor;
    private doubleSolenoid shifter2;

    public static IntakeWheelsStates state = IntakeWheelsStates.Off;

    public IntakeWheels(WPI_VictorSPX intakeMotor, doubleSolenoid shifter2) {
        this.intakeMotor = intakeMotor;
        this.shifter2 = shifter2;
    }

    public enum IntakeWheelsStates {
        Off, Intake;
    }
        
    public boolean Intake(){
        return Hardware.driverPad.getRawButton(Constants.INTAKE);
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
                break;
            case Intake:
                intakeMotor.set(1);
                SmartDashboard.putString("Intake", "Intake");
                shifter2.open();
                if(!Intake()) {
                    newState = IntakeWheelsStates.Off; 
                }
                break;

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