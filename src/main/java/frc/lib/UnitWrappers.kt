package frc.lib

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.Centimeters
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.units.Units.DegreesPerSecondPerSecond
import edu.wpi.first.units.Units.Fahrenheit
import edu.wpi.first.units.Units.Feet
import edu.wpi.first.units.Units.FeetPerSecond
import edu.wpi.first.units.Units.FeetPerSecondPerSecond
import edu.wpi.first.units.Units.Grams
import edu.wpi.first.units.Units.Gs
import edu.wpi.first.units.Units.Hertz
import edu.wpi.first.units.Units.Horsepower
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.InchesPerSecond
import edu.wpi.first.units.Units.InchesPerSecondPerSecond
import edu.wpi.first.units.Units.Joules
import edu.wpi.first.units.Units.Kelvin
import edu.wpi.first.units.Units.KiloOhms
import edu.wpi.first.units.Units.KilogramMetersPerSecond
import edu.wpi.first.units.Units.KilogramMetersSquaredPerSecond
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.Kilograms
import edu.wpi.first.units.Units.Kilojoules
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.MetersPerSecondPerSecond
import edu.wpi.first.units.Units.Microsecond
import edu.wpi.first.units.Units.Microseconds
import edu.wpi.first.units.Units.MilliOhms
import edu.wpi.first.units.Units.Milliamps
import edu.wpi.first.units.Units.Millihertz
import edu.wpi.first.units.Units.Millijoules
import edu.wpi.first.units.Units.Millimeters
import edu.wpi.first.units.Units.Millisecond
import edu.wpi.first.units.Units.Milliseconds
import edu.wpi.first.units.Units.Millivolts
import edu.wpi.first.units.Units.Milliwatts
import edu.wpi.first.units.Units.Minute
import edu.wpi.first.units.Units.Minutes
import edu.wpi.first.units.Units.Newton
import edu.wpi.first.units.Units.NewtonMeter
import edu.wpi.first.units.Units.NewtonMeters
import edu.wpi.first.units.Units.Newtons
import edu.wpi.first.units.Units.Ohms
import edu.wpi.first.units.Units.OunceForce
import edu.wpi.first.units.Units.OunceInch
import edu.wpi.first.units.Units.OunceInches
import edu.wpi.first.units.Units.Ounces
import edu.wpi.first.units.Units.OuncesForce
import edu.wpi.first.units.Units.Percent
import edu.wpi.first.units.Units.PoundFeet
import edu.wpi.first.units.Units.PoundFoot
import edu.wpi.first.units.Units.PoundForce
import edu.wpi.first.units.Units.PoundInch
import edu.wpi.first.units.Units.PoundInches
import edu.wpi.first.units.Units.Pounds
import edu.wpi.first.units.Units.PoundsForce
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.RadiansPerSecondPerSecond
import edu.wpi.first.units.Units.Revolutions
import edu.wpi.first.units.Units.RevolutionsPerSecond
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.Units.RotationsPerSecondPerSecond
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Value
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.Units.VoltsPerMeterPerSecond
import edu.wpi.first.units.Units.VoltsPerMeterPerSecondSquared
import edu.wpi.first.units.Units.VoltsPerRadianPerSecond
import edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared
import edu.wpi.first.units.Units.Watts

val Number.value get() = Value.of(toDouble())!!
val Number.percent get() = Percent.of(toDouble())!!
val Number.meters get() = Meters.of(toDouble())!!
val Number.millimeters get() = Millimeters.of(toDouble())!!
val Number.centimeters get() = Centimeters.of(toDouble())!!
val Number.inches get() = Inches.of(toDouble())!!
val Number.feet get() = Feet.of(toDouble())!!
val Number.seconds get() = Seconds.of(toDouble())!!
val Number.milliseconds get() = Milliseconds.of(toDouble())!!
val Number.millisecond get() = Millisecond.of(toDouble())!!
val Number.microseconds get() = Microseconds.of(toDouble())!!
val Number.microsecond get() = Microsecond.of(toDouble())!!
val Number.minutes get() = Minutes.of(toDouble())!!
val Number.minute get() = Minute.of(toDouble())!!
val Number.radians get() = Radians.of(toDouble())!!
val Number.revolutions get() = Revolutions.of(toDouble())!!
val Number.rotations get() = Rotations.of(toDouble())!!
val Number.degrees get() = Degrees.of(toDouble())!!
val Number.metersPerSecond get() = MetersPerSecond.of(toDouble())!!
val Number.feetPerSecond get() = FeetPerSecond.of(toDouble())!!
val Number.inchesPerSecond get() = InchesPerSecond.of(toDouble())!!
val Number.revolutionsPerSecond get() = RevolutionsPerSecond.of(toDouble())!!
val Number.rotationsPerSecond get() = RotationsPerSecond.of(toDouble())!!
val Number.rPM get() = RPM.of(toDouble())!!
val Number.radiansPerSecond get() = RadiansPerSecond.of(toDouble())!!
val Number.degreesPerSecond get() = DegreesPerSecond.of(toDouble())!!
val Number.hertz get() = Hertz.of(toDouble())!!
val Number.millihertz get() = Millihertz.of(toDouble())!!
val Number.metersPerSecondPerSecond get() = MetersPerSecondPerSecond.of(toDouble())!!
val Number.feetPerSecondPerSecond get() = FeetPerSecondPerSecond.of(toDouble())!!
val Number.inchesPerSecondPerSecond get() = InchesPerSecondPerSecond.of(toDouble())!!
val Number.rotationsPerSecondPerSecond get() = RotationsPerSecondPerSecond.of(toDouble())!!
val Number.radiansPerSecondPerSecond get() = RadiansPerSecondPerSecond.of(toDouble())!!
val Number.degreesPerSecondPerSecond get() = DegreesPerSecondPerSecond.of(toDouble())!!
val Number.gs get() = Gs.of(toDouble())!!
val Number.kilograms get() = Kilograms.of(toDouble())!!
val Number.grams get() = Grams.of(toDouble())!!
val Number.pounds get() = Pounds.of(toDouble())!!
val Number.ounces get() = Ounces.of(toDouble())!!
val Number.newtons get() = Newtons.of(toDouble())!!
val Number.newton get() = Newton.of(toDouble())!!
val Number.poundsForce get() = PoundsForce.of(toDouble())!!
val Number.poundForce get() = PoundForce.of(toDouble())!!
val Number.ouncesForce get() = OuncesForce.of(toDouble())!!
val Number.ounceForce get() = OunceForce.of(toDouble())!!
val Number.newtonMeters get() = NewtonMeters.of(toDouble())!!
val Number.newtonMeter get() = NewtonMeter.of(toDouble())!!
val Number.poundFeet get() = PoundFeet.of(toDouble())!!
val Number.poundFoot get() = PoundFoot.of(toDouble())!!
val Number.poundInches get() = PoundInches.of(toDouble())!!
val Number.poundInch get() = PoundInch.of(toDouble())!!
val Number.ounceInches get() = OunceInches.of(toDouble())!!
val Number.ounceInch get() = OunceInch.of(toDouble())!!
val Number.kilogramMetersPerSecond get() = KilogramMetersPerSecond.of(toDouble())!!
val Number.kilogramMetersSquaredPerSecond get() = KilogramMetersSquaredPerSecond.of(toDouble())!!
val Number.kilogramSquareMeters get() = KilogramSquareMeters.of(toDouble())!!
val Number.volts get() = Volts.of(toDouble())!!
val Number.millivolts get() = Millivolts.of(toDouble())!!
val Number.amps get() = Amps.of(toDouble())!!
val Number.milliamps get() = Milliamps.of(toDouble())!!
val Number.ohms get() = Ohms.of(toDouble())!!
val Number.kiloOhms get() = KiloOhms.of(toDouble())!!
val Number.milliOhms get() = MilliOhms.of(toDouble())!!
val Number.joules get() = Joules.of(toDouble())!!
val Number.millijoules get() = Millijoules.of(toDouble())!!
val Number.kilojoules get() = Kilojoules.of(toDouble())!!
val Number.watts get() = Watts.of(toDouble())!!
val Number.milliwatts get() = Milliwatts.of(toDouble())!!
val Number.horsepower get() = Horsepower.of(toDouble())!!
val Number.kelvin get() = Kelvin.of(toDouble())!!
val Number.celsius get() = Celsius.of(toDouble())!!
val Number.fahrenheit get() = Fahrenheit.of(toDouble())!!
val Number.voltsPerMeterPerSecond get() = VoltsPerMeterPerSecond.of(toDouble())!!
val Number.voltsPerMeterPerSecondSquared get() = VoltsPerMeterPerSecondSquared.of(toDouble())!!
val Number.voltsPerRadianPerSecond get() = VoltsPerRadianPerSecond.of(toDouble())!!
val Number.voltsPerRadianPerSecondSquared get() = VoltsPerRadianPerSecondSquared.of(toDouble())!!
