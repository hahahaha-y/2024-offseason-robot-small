package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import frc.robot.RobotConstants;

import static edu.wpi.first.units.Units.*;

public class ShooterIOTalonFX implements ShooterIO{

    private final TalonFX shooterHighMotor =
            new TalonFX(RobotConstants.ShooterConstants.SHOOTER_MOTORH_ID, RobotConstants.CAN_BUS_NAME);
    private final TalonFX shooterLowMotor =
            new TalonFX(RobotConstants.ShooterConstants.SHOOTER_MOTORL_ID, RobotConstants.CAN_BUS_NAME);

    public ShooterIOTalonFX() {
        shooterHighMotor.getConfigurator().apply(new Slot0Configs()
                .withKP(0.25)
                .withKI(0.0)
                .withKD(0.001)
                .withKA(0.0037512677)
                .withKV(0.115)
                .withKS(0.28475008));
        shooterLowMotor.getConfigurator().apply(new Slot0Configs()
                .withKP(0.25)
                .withKI(0.0)
                .withKD(0.001)
                .withKA(0.0037512677)
                .withKV(0.115)
                .withKS(0.28475008));

        shooterLowMotor.setInverted(true);
        shooterHighMotor.setInverted(true);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterConnected = BaseStatusSignal.refreshAll(
                shooterHighMotor.getVelocity(),
                shooterHighMotor.getMotorVoltage(),
                shooterHighMotor.getSupplyCurrent(),
                shooterLowMotor.getVelocity(),
                shooterLowMotor.getMotorVoltage(),
                shooterLowMotor.getSupplyCurrent()
        ).isOK();
        inputs.shooterHighSpeed = RotationsPerSecond.of(shooterHighMotor.getVelocity().getValueAsDouble());
        inputs.shooterHighSupplyCurrent = Amps.of(shooterHighMotor.getSupplyCurrent().getValueAsDouble());
        inputs.shooterHighVoltage = Volts.of(shooterHighMotor.getMotorVoltage().getValueAsDouble());
        inputs.shooterLowSpeed = RotationsPerSecond.of(shooterHighMotor.getVelocity().getValueAsDouble());
        inputs.shooterLowSupplyCurrent = Amps.of(shooterHighMotor.getSupplyCurrent().getValueAsDouble());
        inputs.shooterLowVoltage = Volts.of(shooterHighMotor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocityRPM){
        shooterHighMotor.setControl(new VelocityVoltage(
                velocityRPM.magnitude() / 60,
                0.0,
                true,
                0,
                0,
                false,
                false,
                false
        ));
        shooterHighMotor.setControl(new VelocityVoltage(
                velocityRPM.magnitude() / 60,
                0.0,
                true,
                0,
                0,
                false,
                false,
                false
        ));
    }

    public void setVelocity(Measure<Velocity<Angle>> highRPM,Measure<Velocity<Angle>> lowRPM){
        shooterHighMotor.setControl(new VelocityVoltage(
                highRPM.magnitude() / 60,
                0.0,
                true,
                0,
                0,
                false,
                false,
                false
        ));
        shooterHighMotor.setControl(new VelocityVoltage(
                lowRPM.magnitude() / 60,
                0.0,
                true,
                0,
                0,
                false,
                false,
                false
        ));
    }

    @Override
    public void setVoltage(Measure<Voltage> voltage){
        shooterHighMotor.setVoltage(voltage.magnitude());
        shooterLowMotor.setVoltage(voltage.magnitude());
    }
}
