package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ShooterIO {

    default void updateInputs(ShooterIOInputs inputs) {
    }

    default void setVelocity(Measure<Velocity<Angle>> velocity) {
    }

    default void setVoltage(Measure<Voltage> voltage) {
    }

    @AutoLog
    class ShooterIOInputs{
        public boolean shooterConnected = true;
        public Measure<Voltage> shooterHighVoltage = Volts.zero();
        public Measure<Velocity<Angle>> shooterHighSpeed = RotationsPerSecond.zero();
        public Measure<Current> shooterHighSupplyCurrent = Amps.zero();
        public Measure<Voltage> shooterLowVoltage = Volts.zero();
        public Measure<Velocity<Angle>> shooterLowSpeed = RotationsPerSecond.zero();
        public Measure<Current> shooterLowSupplyCurrent = Amps.zero();
    }
}
