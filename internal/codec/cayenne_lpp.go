package codec

import (
	"bytes"
	"encoding/binary"
	"encoding/gob"
	"fmt"
	"io"

	"github.com/pkg/errors"
)

func init() {
	gob.Register(CayenneLPP{})
}

// CayenneLPP types.
const (
	lppDigitalInput      byte = 0
	lppDigitalOutput     byte = 1
	lppAnalogInput       byte = 2
	lppAnalogOutput      byte = 3
	lppGenericSensor 	 byte = 100		/* custom type */
	lppIlluminanceSensor byte = 101
	lppPresenceSensor    byte = 102
	lppTemperatureSensor byte = 103
	lppHumiditySensor    byte = 104
	lppPowerMeasurement  byte = 105		/* custom type */
	lppActuation		 byte = 106		/* custom type */
	lppSetPoint			 byte = 108		/* custom type */
	lppLoadControl		 byte = 110		/* custom type */
	lppLightControl		 byte = 111		/* custom type */
	lppPowerControl		 byte = 112		/* custom type */
	lppAccelerometer     byte = 113
	lppMagnetometer		 byte = 114		/* custom type */
	lppBarometer         byte = 115
	lppVoltage			 byte = 116		/* custom type */
	lppCurrent			 byte = 117		/* custom type */
	lppFrequency		 byte = 118		/* custom type */
	lppPercentage		 byte = 120		/* custom type */
	lppAltitude			 byte = 121		/* custom type */
	lppLoad		 		 byte = 122		/* custom type */
	lppPressure			 byte = 123		/* custom type */
	lppLoudness			 byte = 124		/* custom type */
	lppConcentration	 byte = 125		/* custom type */
	lppAcidity			 byte = 126		/* custom type */
	lppConductivity		 byte = 127		/* custom type */
	lppPower			 byte = 128		/* custom type */
	lppDistance			 byte = 130		/* custom type */
	lppEnergy		 	 byte = 131		/* custom type */
	lppDirection		 byte = 132		/* custom type */
	lppTime				 byte = 133		/* custom type */
	lppGyrometer         byte = 134
	lppColour			 byte = 135		/* custom type */
	lppGPSLocation       byte = 136
	lppPositioner		 byte = 137		/* custom type */
	lppSwitch			 byte = 142		/* custom type */
	lppLevelControl		 byte = 143		/* custom type */
	lppUpDownControl	 byte = 144		/* custom type */
	lppMultipleAxisJoystick byte = 145	/* custom type */
	lppRate		 		 byte = 146		/* custom type */
	lppPushButton 		 byte = 147		/* custom type */
	lppMultistateSelector byte = 148	/* custom type */
	lppMoisture 		 byte = 170		/* custom type */
	lppSmoke	 		 byte = 171		/* custom type */
	lppAlcohol	 		 byte = 172		/* custom type */
	lppLPG 		 		 byte = 173		/* custom type */
	lppCarbonMonoxide	 byte = 174		/* custom type */
	lppCarbonDioxide	 byte = 175		/* custom type */
	lppAirQuality 		 byte = 176		/* custom type */
	lppCollision	 	 byte = 177		/* custom type */
	lppDust				 byte = 178		/* custom type */
	lppFire				 byte = 179		/* custom type */
	lppUV		 		 byte = 180		/* custom type */
	lppBattery			 byte = 181		/* custom type */
	lppVelocity			 byte = 182		/* custom type */


)

// Accelerometer defines the accelerometer data.
type Accelerometer struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// Gyrometer defines the gyrometer data.
type Gyrometer struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// GPSLocation defines the GPS location data.
type GPSLocation struct {
	Latitude  float64 `json:"latitude"`
	Longitude float64 `json:"longitude"`
	Altitude  float64 `json:"altitude"`
}

// Magnetometer defines the magnetometer/compass data.
type Magnetometer struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// MultipleAxisJoystick defines the input data.
type MultipleAxisJoystick struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// CayenneLPP defines the Cayenne LPP data structure.
type CayenneLPP struct {
	DigitalInput      map[byte]uint8         `json:"digitalInput,omitempty" influxdb:"digital_input"`
	DigitalOutput     map[byte]uint8         `json:"digitalOutput,omitempty" influxdb:"digital_output"`
	AnalogInput       map[byte]float64       `json:"analogInput,omitempty" influxdb:"analog_input"`
	AnalogOutput      map[byte]float64       `json:"analogOutput,omitempty" influxdb:"analog_output"`
	GenericSensor	  map[byte]float64
	IlluminanceSensor map[byte]uint16        `json:"illuminanceSensor,omitempty" influxdb:"illuminance_sensor"`
	PresenceSensor    map[byte]uint8         `json:"presenceSensor,omitempty" influxdb:"presence_sensor"`
	TemperatureSensor map[byte]float64       `json:"temperatureSensor,omitempty" influxdb:"temperature_sensor"`
	HumiditySensor    map[byte]float64       `json:"humiditySensor,omitempty" influxdb:"humidity_sensor"`
	PowerMeasurement  map[byte]float64
	Actuation	  	  map[byte]float64
	SetPoint		  map[byte]float64
	LoadControl		  map[byte]float64
	LightControl	  map[byte]float64
	PowerControl	  map[byte]float64
	Accelerometer     map[byte]Accelerometer `json:"accelerometer,omitempty" influxdb:"accelerometer"`
	Magnetometer	  map[byte]Magnetometer
	Barometer         map[byte]float64       `json:"barometer,omitempty" influxdb:"barometer"`
	Voltage			  map[byte]float64
	Current			  map[byte]float64
	Frequency		  map[byte]float64
	Percentage		  map[byte]float64
	Altitude		  map[byte]float64
	Load			  map[byte]float64
	Pressure		  map[byte]float64
	Loudness		  map[byte]float64
	Concentration	  map[byte]float64
	Acidity		 	  map[byte]float64
	Conductivity	  map[byte]float64
	Power			  map[byte]float64
	Distance		  map[byte]float64
	Energy			  map[byte]float64
	Direction		  map[byte]float64
	Time			  map[byte]float64
	Gyrometer         map[byte]Gyrometer     `json:"gyrometer,omitempty" influxdb:"gyrometer"`
	Colour			  map[byte]float64
	GPSLocation       map[byte]GPSLocation   `json:"gpsLocation,omitempty" influxdb:"gps_location"`
	Positioner		  map[byte]float64
	Switch		  	  map[byte]float64
	LevelControl	  map[byte]float64
	UpDownControl	  map[byte]float64
	MultipleAxisJoystick map[byte]MultipleAxisJoystick
	Rate		  	  map[byte]float64
	PushButton		  map[byte]uint8
	MultistateSelector map[byte]uint16
	Moisture		  map[byte]float64
	Smoke			  map[byte]float64
	Alcohol			  map[byte]float64
	LPG		  		  map[byte]float64
	CarbonMonoxide	  map[byte]float64
	CarbonDioxide	  map[byte]float64
	AirQuality		  map[byte]float64
	Collision		  map[byte]float64
	Dust			  map[byte]float64
	Fire			  map[byte]float64
	UV				  map[byte]float64
	Battery			  map[byte]float64
	Velocity		  map[byte]float64
}

// Object returns the CayenneLPP data object.
func (c CayenneLPP) Object() interface{} {
	return c
}

// DecodeBytes decodes the payload from a slice of bytes.
func (c *CayenneLPP) DecodeBytes(data []byte) error {
	var err error
	buf := make([]byte, 2)
	r := bytes.NewReader(data)

	for {
		_, err = io.ReadFull(r, buf)
		if err != nil {
			if err == io.EOF {
				break
			}
			return errors.Wrap(err, "read full error")
		}

		switch buf[1] {
		case lppDigitalInput:
			err = lppDigitalInputDecode(buf[0], r, c)
		case lppDigitalOutput:
			err = lppDigitalOutputDecode(buf[0], r, c)
		case lppAnalogInput:
			err = lppAnalogInputDecode(buf[0], r, c)
		case lppAnalogOutput:
			err = lppAnalogOutputDecode(buf[0], r, c)
		case lppGenericSensor:  /* custom type*/
			err = lppGenericSensorDecode(buf[0], r, c)
		case lppIlluminanceSensor:
			err = lppIlluminanceSensorDecode(buf[0], r, c)
		case lppPresenceSensor:
			err = lppPresenceSensorDecode(buf[0], r, c)
		case lppTemperatureSensor:
			err = lppTemperatureSensorDecode(buf[0], r, c)
		case lppHumiditySensor:
			err = lppHumiditySensorDecode(buf[0], r, c)
		case lppPowerMeasurement:  /* custom type*/
			err = lppPowerMeasurementDecode(buf[0], r, c)	
		case lppActuation:  /* custom type*/
			err = lppActuationDecode(buf[0], r, c)	
		case lppSetPoint:  /* custom type*/
			err = lppSetPointDecode(buf[0], r, c)	
		case lppLoadControl:  /* custom type*/
			err = lppLoadControlDecode(buf[0], r, c)	
		case lppLightControl:  /* custom type*/
			err = lppLightControlDecode(buf[0], r, c)	
		case lppPowerControl:  /* custom type*/
			err = lppPowerControlDecode(buf[0], r, c)	
		case lppAccelerometer:
			err = lppAccelerometerDecode(buf[0], r, c)
		case lppMagnetometer:  /* custom type*/
			err = lppMagnetometerDecode(buf[0], r, c)			
		case lppBarometer:
			err = lppBarometerDecode(buf[0], r, c)
		case lppVoltage:  /* custom type*/
			err = lppVoltageDecode(buf[0], r, c)
		case lppCurrent:  /* custom type*/
			err = lppCurrentDecode(buf[0], r, c)
		case lppFrequency:  /* custom type*/
			err = lppFrequencyDecode(buf[0], r, c)
		case lppPercentage:  /* custom type*/
			err = lppPercentageDecode(buf[0], r, c)
		case lppAltitude:  /* custom type*/
			err = lppAltitudeDecode(buf[0], r, c)
		case lppLoad:  /* custom type*/
			err = lppLoadDecode(buf[0], r, c)
		case lppPressure:  /* custom type*/
			err = lppPressureDecode(buf[0], r, c)
		case lppLoudness:  /* custom type*/
			err = lppLoudnessDecode(buf[0], r, c)
		case lppConcentration:  /* custom type*/
			err = lppConcentrationDecode(buf[0], r, c)
		case lppAcidity:  /* custom type*/
			err = lppAcidityDecode(buf[0], r, c)
		case lppConductivity:  /* custom type*/
			err = lppConductivityDecode(buf[0], r, c)
		case lppPower:  /* custom type*/
			err = lppPowerDecode(buf[0], r, c)
		case lppDistance:  /* custom type*/
			err = lppDistanceDecode(buf[0], r, c)
		case lppEnergy:  /* custom type*/
			err = lppEnergyDecode(buf[0], r, c)
		case lppDirection:  /* custom type*/
			err = lppDirectionDecode(buf[0], r, c)
		case lppTime:  /* custom type*/
			err = lppTimeDecode(buf[0], r, c)
		case lppGyrometer:
			err = lppGyrometerDecode(buf[0], r, c)
		case lppColour:  /* custom type*/
			err = lppColourDecode(buf[0], r, c)
		case lppGPSLocation:
			err = lppGPSLocationDecode(buf[0], r, c)
		case lppPositioner:  /* custom type*/
			err = lppPositionerDecode(buf[0], r, c)
		case lppSwitch:  /* custom type*/
			err = lppSwitchDecode(buf[0], r, c)
		case lppLevelControl:  /* custom type*/
			err = lppLevelControlDecode(buf[0], r, c)
		case lppUpDownControl:  /* custom type*/
			err = lppUpDownControlDecode(buf[0], r, c)
		case lppMultipleAxisJoystick:  /* custom type*/
			err = lppMultipleAxisJoystickDecode(buf[0], r, c)
		case lppRate:  /* custom type*/
			err = lppRateDecode(buf[0], r, c)
		case lppPushButton:  /* custom type*/
			err = lppPushButtonDecode(buf[0], r, c)
		case lppMultistateSelector:  /* custom type*/
			err = lppMultistateSelectorDecode(buf[0], r, c)
		case lppMoisture:  /* custom type*/
			err = lppMoistureDecode(buf[0], r, c)
		case lppSmoke:  /* custom type*/
			err = lppSmokeDecode(buf[0], r, c)
		case lppAlcohol:  /* custom type*/
			err = lppAlcoholDecode(buf[0], r, c)
		case lppLPG:  /* custom type*/
			err = lppLPGDecode(buf[0], r, c)
		case lppCarbonMonoxide:  /* custom type*/
			err = lppCarbonMonoxideDecode(buf[0], r, c)
		case lppCarbonDioxide:  /* custom type*/
			err = lppCarbonDioxideDecode(buf[0], r, c)
		case lppAirQuality:  /* custom type*/
			err = lppAirQualityDecode(buf[0], r, c)
		case lppCollision:  /* custom type*/
			err = lppCollisionDecode(buf[0], r, c)
		case lppDust:  /* custom type*/
			err = lppDustDecode(buf[0], r, c)
		case lppFire:  /* custom type*/
			err = lppFireDecode(buf[0], r, c)
		case lppUV:  /* custom type*/
			err = lppUVDecode(buf[0], r, c)
		case lppBattery:  /* custom type*/
			err = lppBatteryDecode(buf[0], r, c)
		case lppVelocity:  /* custom type*/
			err = lppVelocityDecode(buf[0], r, c)
		default:
			return fmt.Errorf("invalid data type: %d", buf[1])
		}

		if err != nil {
			return errors.Wrap(err, "decode error")
		}
	}

	return nil
}

// EncodeToBytes encodes the payload to a slice of bytes.
func (c CayenneLPP) EncodeToBytes() ([]byte, error) {
	w := bytes.NewBuffer([]byte{})

	for k, v := range c.DigitalInput {
		if err := lppDigitalInputEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.DigitalOutput {
		if err := lppDigitalOutputEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.AnalogInput {
		if err := lppAnalogInputEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.AnalogOutput {
		if err := lppAnalogOutputEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.GenericSensor {
		if err := lppGenericSensorEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.IlluminanceSensor {
		if err := lppIlluminanceSensorEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.PresenceSensor {
		if err := lppPresenceSensorEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.TemperatureSensor {
		if err := lppTemperatureSensorEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.HumiditySensor {
		if err := lppHumiditySensorEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.PowerMeasurement {
		if err := lppPowerMeasurementEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Actuation {
		if err := lppActuationEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.SetPoint {
		if err := lppSetPointEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.LoadControl {
		if err := lppLoadControlEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.LightControl {
		if err := lppLightControlEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.PowerControl {
		if err := lppPowerControlEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Accelerometer {
		if err := lppAccelerometerEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Magnetometer {
		if err := lppMagnetometerEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Barometer {
		if err := lppBarometerEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Voltage {
		if err := lppVoltageEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Current {
		if err := lppCurrentEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Frequency {
		if err := lppFrequencyEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Percentage {
		if err := lppPercentageEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Altitude {
		if err := lppAltitudeEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Load {
		if err := lppLoadEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Pressure {
		if err := lppPressureEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Loudness {
		if err := lppLoudnessEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Concentration {
		if err := lppConcentrationEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Acidity {
		if err := lppAcidityEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Conductivity {
		if err := lppConductivityEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Power {
		if err := lppPowerEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Distance {
		if err := lppDistanceEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Energy {
		if err := lppEnergyEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Direction {
		if err := lppDirectionEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Time {
		if err := lppTimeEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Gyrometer {
		if err := lppGyrometerEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Colour {
		if err := lppColourEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.GPSLocation {
		if err := lppGPSLocationEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Positioner {
		if err := lppPositionerEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Switch {
		if err := lppSwitchEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.LevelControl {
		if err := lppLevelControlEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.UpDownControl {
		if err := lppUpDownControlEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.MultipleAxisJoystick {
		if err := lppMultipleAxisJoystickEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Rate {
		if err := lppRateEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.PushButton {
		if err := lppPushButtonEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.MultistateSelector {
		if err := lppMultistateSelectorEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Moisture {
		if err := lppMoistureEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Smoke {
		if err := lppSmokeEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Alcohol {
		if err := lppAlcoholEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.LPG {
		if err := lppLPGEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.CarbonMonoxide {
		if err := lppCarbonMonoxideEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.CarbonDioxide {
		if err := lppCarbonDioxideEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.AirQuality {
		if err := lppAirQualityEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Collision {
		if err := lppCollisionEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Dust {
		if err := lppDustEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Fire {
		if err := lppFireEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.UV {
		if err := lppUVEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Battery {
		if err := lppBatteryEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	for k, v := range c.Velocity {
		if err := lppVelocityEncode(k, w, v); err != nil {
			return nil, err
		}
	}
	return w.Bytes(), nil
}

func lppDigitalInputDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var b uint8
	if err := binary.Read(r, binary.BigEndian, &b); err != nil {
		return errors.Wrap(err, "read uint8 error")
	}
	if out.DigitalInput == nil {
		out.DigitalInput = make(map[uint8]uint8)
	}
	out.DigitalInput[channel] = b
	return nil
}

func lppDigitalInputEncode(channel uint8, w io.Writer, data uint8) error {
	w.Write([]byte{channel, lppDigitalInput})
	if err := binary.Write(w, binary.BigEndian, data); err != nil {
		return errors.Wrap(err, "write uint8 error")
	}
	return nil
}

func lppDigitalOutputDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var b uint8
	if err := binary.Read(r, binary.BigEndian, &b); err != nil {
		return errors.Wrap(err, "read uint8 error")
	}
	if out.DigitalOutput == nil {
		out.DigitalOutput = make(map[uint8]uint8)
	}
	out.DigitalOutput[channel] = b
	return nil
}

func lppDigitalOutputEncode(channel uint8, w io.Writer, data uint8) error {
	w.Write([]byte{channel, lppDigitalOutput})
	if err := binary.Write(w, binary.BigEndian, data); err != nil {
		return errors.Wrap(err, "write uint8 error")
	}
	return nil
}

func lppAnalogInputDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var analog int16
	if err := binary.Read(r, binary.BigEndian, &analog); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.AnalogInput == nil {
		out.AnalogInput = make(map[uint8]float64)
	}
	out.AnalogInput[channel] = float64(analog) / 100
	return nil
}

func lppAnalogInputEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppAnalogInput})
	if err := binary.Write(w, binary.BigEndian, int16(data*100)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppAnalogOutputDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var analog int16
	if err := binary.Read(r, binary.BigEndian, &analog); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.AnalogOutput == nil {
		out.AnalogOutput = make(map[uint8]float64)
	}
	out.AnalogOutput[channel] = float64(analog) / 100
	return nil
}

func lppAnalogOutputEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppAnalogOutput})
	if err := binary.Write(w, binary.BigEndian, int16(data*100)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppGenericSensorDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val uint16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read uint16 error")
	}
	if out.GenericSensor == nil {
		out.GenericSensor = make(map[uint8]float64)
	}
	out.GenericSensor[channel] = float64(val)
	return nil
}

func lppGenericSensorEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppGenericSensor})
	if err := binary.Write(w, binary.BigEndian, int16(data)); err != nil {
		return errors.Wrap(err, "write uint16 error")
	}
	return nil
}

func lppIlluminanceSensorDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var illum uint16
	if err := binary.Read(r, binary.BigEndian, &illum); err != nil {
		return errors.Wrap(err, "read uint16 error")
	}
	if out.IlluminanceSensor == nil {
		out.IlluminanceSensor = make(map[uint8]uint16)
	}
	out.IlluminanceSensor[channel] = illum
	return nil
}

func lppIlluminanceSensorEncode(channel uint8, w io.Writer, data uint16) error {
	w.Write([]byte{channel, lppIlluminanceSensor})
	if err := binary.Write(w, binary.BigEndian, int16(data)); err != nil {
		return errors.Wrap(err, "write uint16 error")
	}
	return nil
}

func lppPresenceSensorDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var b uint8
	if err := binary.Read(r, binary.BigEndian, &b); err != nil {
		return errors.Wrap(err, "read uint8 error")
	}
	if out.PresenceSensor == nil {
		out.PresenceSensor = make(map[uint8]uint8)
	}
	out.PresenceSensor[channel] = b
	return nil
}

func lppPresenceSensorEncode(channel uint8, w io.Writer, data uint8) error {
	w.Write([]byte{channel, lppPresenceSensor})
	if err := binary.Write(w, binary.BigEndian, data); err != nil {
		return errors.Wrap(err, "write uint8 error")
	}
	return nil
}

func lppTemperatureSensorDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var temp int16
	if err := binary.Read(r, binary.BigEndian, &temp); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.TemperatureSensor == nil {
		out.TemperatureSensor = make(map[uint8]float64)
	}
	out.TemperatureSensor[channel] = float64(temp) / 10
	return nil
}

func lppTemperatureSensorEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppTemperatureSensor})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppHumiditySensorDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var b uint8
	if err := binary.Read(r, binary.BigEndian, &b); err != nil {
		return errors.Wrap(err, "read uint8 error")
	}
	if out.HumiditySensor == nil {
		out.HumiditySensor = make(map[uint8]float64)
	}
	out.HumiditySensor[channel] = float64(b) / 2
	return nil
}

func lppHumiditySensorEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppHumiditySensor})
	if err := binary.Write(w, binary.BigEndian, uint8(data*2)); err != nil {
		return errors.Wrap(err, "write uint8 error")
	}
	return nil
}

func lppPowerMeasurementDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.PowerMeasurement == nil {
		out.PowerMeasurement = make(map[uint8]float64)
	}
	out.PowerMeasurement[channel] = float64(val) / 10
	return nil
}

func lppPowerMeasurementEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppPowerMeasurement})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppActuationDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Actuation == nil {
		out.Actuation = make(map[uint8]float64)
	}
	out.Actuation[channel] = float64(val)
	return nil
}

func lppActuationEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppActuation})
	if err := binary.Write(w, binary.BigEndian, int16(data)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppSetPointDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.SetPoint == nil {
		out.SetPoint = make(map[uint8]float64)
	}
	out.SetPoint[channel] = float64(val)
	return nil
}

func lppSetPointEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppSetPoint})
	if err := binary.Write(w, binary.BigEndian, int16(data)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppLoadControlDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var temp int16
	if err := binary.Read(r, binary.BigEndian, &temp); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.LoadControl == nil {
		out.LoadControl = make(map[uint8]float64)
	}
	out.LoadControl[channel] = float64(temp) / 10
	return nil
}

func lppLoadControlEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppLoadControl})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppLightControlDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var temp int16
	if err := binary.Read(r, binary.BigEndian, &temp); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.LightControl == nil {
		out.LightControl = make(map[uint8]float64)
	}
	out.LightControl[channel] = float64(temp) / 10
	return nil
}

func lppLightControlEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppLightControl})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppPowerControlDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var temp int16
	if err := binary.Read(r, binary.BigEndian, &temp); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.PowerControl == nil {
		out.PowerControl = make(map[uint8]float64)
	}
	out.PowerControl[channel] = float64(temp) / 10
	return nil
}

func lppPowerControlEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppPowerControl})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppAccelerometerDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	data := make([]int16, 3)
	for i := range data {
		if err := binary.Read(r, binary.BigEndian, &data[i]); err != nil {
			return errors.Wrap(err, "read int16 error")
		}
	}
	if out.Accelerometer == nil {
		out.Accelerometer = make(map[uint8]Accelerometer)
	}
	out.Accelerometer[channel] = Accelerometer{
		X: float64(data[0]) / 1000,
		Y: float64(data[1]) / 1000,
		Z: float64(data[2]) / 1000,
	}
	return nil
}

func lppAccelerometerEncode(channel uint8, w io.Writer, data Accelerometer) error {
	w.Write([]byte{channel, lppAccelerometer})
	vals := []int16{
		int16(data.X * 1000),
		int16(data.Y * 1000),
		int16(data.Z * 1000),
	}
	for _, v := range vals {
		if err := binary.Write(w, binary.BigEndian, v); err != nil {
			return errors.Wrap(err, "write int16 error")
		}
	}
	return nil
}

func lppMagnetometerDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	data := make([]int16, 3)
	for i := range data {
		if err := binary.Read(r, binary.BigEndian, &data[i]); err != nil {
			return errors.Wrap(err, "read int16 error")
		}
	}
	if out.Magnetometer == nil {
		out.Magnetometer = make(map[uint8]Magnetometer)
	}
	out.Magnetometer[channel] = Magnetometer{
		X: float64(data[0]) / 1000,
		Y: float64(data[1]) / 1000,
		Z: float64(data[2]) / 1000,
	}
	return nil
}

func lppMagnetometerEncode(channel uint8, w io.Writer, data Magnetometer) error {
	w.Write([]byte{channel, lppMagnetometer})
	vals := []int16{
		int16(data.X * 1000),
		int16(data.Y * 1000),
		int16(data.Z * 1000),
	}
	for _, v := range vals {
		if err := binary.Write(w, binary.BigEndian, v); err != nil {
			return errors.Wrap(err, "write int16 error")
		}
	}
	return nil
}

func lppBarometerDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var bar uint16
	if err := binary.Read(r, binary.BigEndian, &bar); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Barometer == nil {
		out.Barometer = make(map[uint8]float64)
	}
	out.Barometer[channel] = float64(bar) / 10
	return nil
}

func lppBarometerEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppBarometer})
	if err := binary.Write(w, binary.BigEndian, uint16(data*10)); err != nil {
		return errors.Wrap(err, "write uint16 error")
	}
	return nil
}

func lppVoltageDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Voltage == nil {
		out.Voltage = make(map[uint8]float64)
	}
	out.Voltage[channel] = float64(val) / 1000
	return nil
}

func lppVoltageEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppVoltage})
	if err := binary.Write(w, binary.BigEndian, int16(data*1000)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppCurrentDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Current == nil {
		out.Current = make(map[uint8]float64)
	}
	out.Current[channel] = float64(val) / 1000
	return nil
}

func lppCurrentEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppCurrent})
	if err := binary.Write(w, binary.BigEndian, int16(data*1000)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppFrequencyDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Frequency == nil {
		out.Frequency = make(map[uint8]float64)
	}
	out.Frequency[channel] = float64(val) / 10
	return nil
}

func lppFrequencyEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppFrequency})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppPercentageDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Percentage == nil {
		out.Percentage = make(map[uint8]float64)
	}
	out.Percentage[channel] = float64(val) / 100
	return nil
}

func lppPercentageEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppPercentage})
	if err := binary.Write(w, binary.BigEndian, int16(data*100)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppAltitudeDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Altitude == nil {
		out.Altitude = make(map[uint8]float64)
	}
	out.Altitude[channel] = float64(val) / 100
	return nil
}

func lppAltitudeEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppAltitude})
	if err := binary.Write(w, binary.BigEndian, int16(data*100)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppLoadDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Load == nil {
		out.Load = make(map[uint8]float64)
	}
	out.Load[channel] = float64(val) / 10
	return nil
}

func lppLoadEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppLoad})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppPressureDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Pressure == nil {
		out.Pressure = make(map[uint8]float64)
	}
	out.Pressure[channel] = float64(val) / 10
	return nil
}

func lppPressureEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppPressure})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppLoudnessDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Loudness == nil {
		out.Loudness = make(map[uint8]float64)
	}
	out.Loudness[channel] = float64(val) / 10
	return nil
}

func lppLoudnessEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppLoudness})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppConcentrationDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Concentration == nil {
		out.Concentration = make(map[uint8]float64)
	}
	out.Concentration[channel] = float64(val) / 10
	return nil
}

func lppConcentrationEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppConcentration})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppAcidityDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Acidity == nil {
		out.Acidity = make(map[uint8]float64)
	}
	out.Acidity[channel] = float64(val) / 10
	return nil
}

func lppAcidityEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppAcidity})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppConductivityDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Conductivity == nil {
		out.Conductivity = make(map[uint8]float64)
	}
	out.Conductivity[channel] = float64(val) / 10
	return nil
}

func lppConductivityEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppConductivity})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppPowerDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Power == nil {
		out.Power = make(map[uint8]float64)
	}
	out.Power[channel] = float64(val) / 10
	return nil
}

func lppPowerEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppPower})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppDistanceDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Distance == nil {
		out.Distance = make(map[uint8]float64)
	}
	out.Distance[channel] = float64(val) / 10
	return nil
}

func lppDistanceEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppDistance})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppEnergyDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Energy == nil {
		out.Energy = make(map[uint8]float64)
	}
	out.Energy[channel] = float64(val) / 10
	return nil
}

func lppEnergyEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppEnergy})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppDirectionDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Direction == nil {
		out.Direction = make(map[uint8]float64)
	}
	out.Direction[channel] = float64(val) / 1000
	return nil
}

func lppDirectionEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppDirection})
	if err := binary.Write(w, binary.BigEndian, int16(data*1000)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppTimeDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Time == nil {
		out.Time = make(map[uint8]float64)
	}
	out.Time[channel] = float64(val) / 1000
	return nil
}

func lppTimeEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppTime})
	if err := binary.Write(w, binary.BigEndian, int16(data*1000)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppGyrometerDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	data := make([]int16, 3)
	for i := range data {
		if err := binary.Read(r, binary.BigEndian, &data[i]); err != nil {
			return errors.Wrap(err, "read int16 error")
		}
	}
	if out.Gyrometer == nil {
		out.Gyrometer = make(map[uint8]Gyrometer)
	}
	out.Gyrometer[channel] = Gyrometer{
		X: float64(data[0]) / 100,
		Y: float64(data[1]) / 100,
		Z: float64(data[2]) / 100,
	}
	return nil
}

func lppGyrometerEncode(channel uint8, w io.Writer, data Gyrometer) error {
	w.Write([]byte{channel, lppGyrometer})
	vals := []int16{
		int16(data.X * 100),
		int16(data.Y * 100),
		int16(data.Z * 100),
	}
	for _, v := range vals {
		if err := binary.Write(w, binary.BigEndian, v); err != nil {
			return errors.Wrap(err, "write int16 error")
		}
	}
	return nil
}

func lppColourDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Colour == nil {
		out.Colour = make(map[uint8]float64)
	}
	out.Colour[channel] = float64(val)
	return nil
}

func lppColourEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppColour})
	if err := binary.Write(w, binary.BigEndian, int16(data)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppGPSLocationDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var lat int32
	var lon int32
	var alt int16
	// data := make([]int32, 3)
	// buf := make([]byte, 9)

	// if _, err := io.ReadFull(r, buf); err != nil {
	// 	return errors.Wrap(err, "read error")
	// }

	// for i := range data {
	// 	b := make([]byte, 4)
	// 	copy(b, buf[i*3:(i*3)+3])
	// 	data[i] = int32(binary.BigEndian.Uint32(b)) >> 8
	// }
	if err := binary.Read(r, binary.BigEndian, &lat); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if err := binary.Read(r, binary.BigEndian, &lon); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if err := binary.Read(r, binary.BigEndian, &alt); err != nil {
		return errors.Wrap(err, "read int16 error")
	}

	if out.GPSLocation == nil {
		out.GPSLocation = make(map[uint8]GPSLocation)
	}
	out.GPSLocation[channel] = GPSLocation{
		Latitude:  float64(lat) / 1000000,
		Longitude: float64(lon) / 1000000,
		Altitude:  float64(alt) / 100,
	}
	return nil
}

func lppGPSLocationEncode(channel uint8, w io.Writer, data GPSLocation) error {
	w.Write([]byte{channel, lppGPSLocation})
	lat := int32(data.Latitude * 1000000)
	lon := int32(data.Longitude * 1000000)
	alt := int16(data.Altitude * 100)
	
	// for _, v := range vals {
	// 	b := make([]byte, 4)
	// 	v = v << 8
	// 	binary.BigEndian.PutUint32(b, uint32(v))
	// 	w.Write(b[0:3])
	// }
	if err := binary.Write(w, binary.BigEndian, lat); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	if err := binary.Write(w, binary.BigEndian, lon); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	if err := binary.Write(w, binary.BigEndian, alt); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppPositionerDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Positioner == nil {
		out.Positioner = make(map[uint8]float64)
	}
	out.Positioner[channel] = float64(val)
	return nil
}

func lppPositionerEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppPositioner})
	if err := binary.Write(w, binary.BigEndian, int16(data)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppSwitchDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Switch == nil {
		out.Switch = make(map[uint8]float64)
	}
	out.Switch[channel] = float64(val)
	return nil
}

func lppSwitchEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppSwitch})
	if err := binary.Write(w, binary.BigEndian, int16(data)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppLevelControlDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.LevelControl == nil {
		out.LevelControl = make(map[uint8]float64)
	}
	out.LevelControl[channel] = float64(val)
	return nil
}

func lppLevelControlEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppLevelControl})
	if err := binary.Write(w, binary.BigEndian, int16(data)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppUpDownControlDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.UpDownControl == nil {
		out.UpDownControl = make(map[uint8]float64)
	}
	out.UpDownControl[channel] = float64(val)
	return nil
}

func lppUpDownControlEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppUpDownControl})
	if err := binary.Write(w, binary.BigEndian, int16(data)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppMultipleAxisJoystickDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	data := make([]int16, 3)
	for i := range data {
		if err := binary.Read(r, binary.BigEndian, &data[i]); err != nil {
			return errors.Wrap(err, "read int16 error")
		}
	}
	if out.MultipleAxisJoystick == nil {
		out.MultipleAxisJoystick = make(map[uint8]MultipleAxisJoystick)
	}
	out.MultipleAxisJoystick[channel] = MultipleAxisJoystick{
		X: float64(data[0]) / 100,
		Y: float64(data[1]) / 100,
		Z: float64(data[2]) / 100,
	}
	return nil
}

func lppMultipleAxisJoystickEncode(channel uint8, w io.Writer, data MultipleAxisJoystick) error {
	w.Write([]byte{channel, lppMultipleAxisJoystick})
	vals := []int16{
		int16(data.X * 100),
		int16(data.Y * 100),
		int16(data.Z * 100),
	}
	for _, v := range vals {
		if err := binary.Write(w, binary.BigEndian, v); err != nil {
			return errors.Wrap(err, "write int16 error")
		}
	}
	return nil
}

func lppRateDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Rate == nil {
		out.Rate = make(map[uint8]float64)
	}
	out.Rate[channel] = float64(val) / 10
	return nil
}

func lppRateEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppRate})
	if err := binary.Write(w, binary.BigEndian, int16(data)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppPushButtonDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var b uint8
	if err := binary.Read(r, binary.BigEndian, &b); err != nil {
		return errors.Wrap(err, "read uint8 error")
	}
	if out.PushButton == nil {
		out.PushButton = make(map[uint8]uint8)
	}
	out.PushButton[channel] = b
	return nil
}

func lppPushButtonEncode(channel uint8, w io.Writer, data uint8) error {
	w.Write([]byte{channel, lppPushButton})
	if err := binary.Write(w, binary.BigEndian, data); err != nil {
		return errors.Wrap(err, "write uint8 error")
	}
	return nil
}

func lppMultistateSelectorDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.MultistateSelector == nil {
		out.MultistateSelector = make(map[uint8]uint16)
	}
	out.MultistateSelector[channel] = uint16(val)
	return nil
}

func lppMultistateSelectorEncode(channel uint8, w io.Writer, data uint16) error {
	w.Write([]byte{channel, lppMultistateSelector})
	if err := binary.Write(w, binary.BigEndian, int16(data)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppMoistureDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Moisture == nil {
		out.Moisture = make(map[uint8]float64)
	}
	out.Moisture[channel] = float64(val) / 2
	return nil
}

func lppMoistureEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppMoisture})
	if err := binary.Write(w, binary.BigEndian, int16(data*2)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppSmokeDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Smoke == nil {
		out.Smoke = make(map[uint8]float64)
	}
	out.Smoke[channel] = float64(val) / 10
	return nil
}

func lppSmokeEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppSmoke})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppAlcoholDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Alcohol == nil {
		out.Alcohol = make(map[uint8]float64)
	}
	out.Alcohol[channel] = float64(val) / 10
	return nil
}

func lppAlcoholEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppAlcohol})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppLPGDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.LPG == nil {
		out.LPG = make(map[uint8]float64)
	}
	out.LPG[channel] = float64(val) / 10
	return nil
}

func lppLPGEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppLPG})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppCarbonMonoxideDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.CarbonMonoxide == nil {
		out.CarbonMonoxide = make(map[uint8]float64)
	}
	out.CarbonMonoxide[channel] = float64(val) / 10
	return nil
}

func lppCarbonMonoxideEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppCarbonMonoxide})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppCarbonDioxideDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.CarbonDioxide == nil {
		out.CarbonDioxide = make(map[uint8]float64)
	}
	out.CarbonDioxide[channel] = float64(val) / 10
	return nil
}

func lppCarbonDioxideEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppCarbonDioxide})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppAirQualityDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.AirQuality == nil {
		out.AirQuality = make(map[uint8]float64)
	}
	out.AirQuality[channel] = float64(val)
	return nil
}

func lppAirQualityEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppAirQuality})
	if err := binary.Write(w, binary.BigEndian, int16(data)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppCollisionDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Collision == nil {
		out.Collision = make(map[uint8]float64)
	}
	out.Collision[channel] = float64(val) / 10
	return nil
}

func lppCollisionEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppCollision})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppDustDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Dust == nil {
		out.Dust = make(map[uint8]float64)
	}
	out.Dust[channel] = float64(val) / 10
	return nil
}

func lppDustEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppDust})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppFireDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Fire == nil {
		out.Fire = make(map[uint8]float64)
	}
	out.Fire[channel] = float64(val) / 10
	return nil
}

func lppFireEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppFire})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppUVDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.UV == nil {
		out.UV = make(map[uint8]float64)
	}
	out.UV[channel] = float64(val) / 10
	return nil
}

func lppUVEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppUV})
	if err := binary.Write(w, binary.BigEndian, int16(data*10)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppBatteryDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Battery == nil {
		out.Battery = make(map[uint8]float64)
	}
	out.Battery[channel] = float64(val) / 2
	return nil
}

func lppBatteryEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppBattery})
	if err := binary.Write(w, binary.BigEndian, int16(data*2)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}

func lppVelocityDecode(channel uint8, r io.Reader, out *CayenneLPP) error {
	var val int16
	if err := binary.Read(r, binary.BigEndian, &val); err != nil {
		return errors.Wrap(err, "read int16 error")
	}
	if out.Velocity == nil {
		out.Velocity = make(map[uint8]float64)
	}
	out.Velocity[channel] = float64(val) / 100
	return nil
}

func lppVelocityEncode(channel uint8, w io.Writer, data float64) error {
	w.Write([]byte{channel, lppVelocity})
	if err := binary.Write(w, binary.BigEndian, int16(data*100)); err != nil {
		return errors.Wrap(err, "write int16 error")
	}
	return nil
}
