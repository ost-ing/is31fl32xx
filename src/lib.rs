//! # IS31FL3205 library
//! A rust-embedded driver for the Lumissil Microsystems IS31FL3205 LED driver

#![no_std]
#![deny(warnings)]

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::{
    blocking::i2c::{Read, Write, WriteRead},
    digital::v2::OutputPin,
};

#[derive(Debug)]
pub enum Error<I> {
    /// I2C bus error
    I2C(I),
    /// Connection error (device not found)
    Conn,
    /// Address error (invalid or out of bounds)
    Address,
    /// Port error (invalid or out of bounds)
    Port,
    /// Configured Interface Error
    InterfaceConfig,
    /// Enable line
    EnableLine,
}

pub enum Register {
    /// Power control register
    /// The Control Register sets software shutdown mode, internal oscillator clock frequency and PWM resolution.
    /// The internal oscillator clock frequency and the PWM resolution will decide the output PWM frequency.
    /// Recommend selecting PWM frequency higher than 20kHz to avoid MLCC audible noise as shown in Table 4.
    PowerControl = 0x00,
    /// Channel [12:1] PWM register byte
    /// Each output has 2 bytes to modulate the PWM duty in 256/1024/4096/65536 steps. If using 8 bit PWM resolution, only PWM_L bits need to be set.
    /// The value of the SL (Scaling Register) decides the peak current of each LED noted as IOUT.
    /// IOUT and the value of the PWM Registers decide the average current of each LED noted as ILED.
    Pwm = 0x07,
    /// Update the PWM and Scaling data
    Update = 0x49,
    /// Update the PWM and Scaling data
    LedScaling = 0x4D,
    /// Control Global DC current/SSD
    GlobalCurrentControl = 0x6E,
    /// Phase Delay and Clock Phase
    PhaseDelayClockPhase = 0x70,
    /// Open short detect enable
    OpenShortDetectEnable = 0x71,
    /// Open short information. TODO
    LedOpenShort = 0x72,
    /// Temperature information. TODO
    TemperatureSensor = 0x77,
    /// Spread spectrum control register. TODO
    SpreadSpectrum = 0x78,
    /// Reset all registers
    Reset = 0x7F,
}

pub enum SoftwareShutdownMode {
    SoftwareShutdown = 0x00,
    Normal = 0x01,
}

pub enum PwmResolution {
    Eightbit = 0x00,
    Tenbit = 0x01,
    TwelveBit = 0x02,
    SixteenBit = 0x03,
}

pub enum OscillatorClock {
    SixteenMHz = 0x00,
    EightMHz = 0x01,
    OneMHz = 0x02,
    FiveHundredKHz = 0x03,
    TwoHundredFiftyKHz = 0x04,
    OneHundredTwentyFiveKHz = 0x05,
    SixtyTwoKHz = 0x06,
    ThirtyOneKHz = 0x07,
}

pub enum OpenShortDetect {
    DetectDisable = 0x00,
    ShortDetectEnable = 0x02,
    OpenDetectEnable = 0x03,
}

pub struct Is31fl3205<I2C, EN> {
    /// `embedded-hal` compatible I2C instance
    interface: Option<I2C>,
    /// Transfer callback
    transfer_callback: Option<fn(addr: u8, data: &[u8])>,
    /// Device address
    address: u8,
    // Enable line
    enable: EN,
}
/// Each of the IS31FL3205 messages are defined by the Message struct.
/// These provide an interface to the transferred data to modify or read registers of the IS31FL3205
pub struct Message {
    /// The register to write or read to/from
    register: Register,
    /// An offset of the base register address
    register_offset: u8,
    /// The data buffer, with a statically allocated 24 bytes
    data: [u8; 24],
    /// The length of the transferred data
    data_length: usize,
}

/// TODO: Add support for advanced features:
/// - phase delay & clock phase
/// - spread spectrum for EMI reduction techniques
/// - open / short functionality and detection
/// - temperature detection
impl Message {
    /// Creates a new Message.
    /// This function should not be called directly
    fn new(register: Register, raw_data: &[u8]) -> Self {
        let mut data = [0u8; 24];
        data[0..raw_data.len()].copy_from_slice(raw_data);
        Self {
            register,
            data,
            data_length: raw_data.len(),
            register_offset: 0,
        }
    }
    /// Defines a register offset, for example when targeting a specific channel
    fn register_offset(mut self, offset: u8) -> Self {
        self.register_offset = offset;
        self
    }
    /// Defines a power control message for initialising oscillator clock, pwm resolution
    /// and device enable / disable
    pub fn power_control(
        osc: OscillatorClock,
        pms: PwmResolution,
        ssd: SoftwareShutdownMode,
    ) -> Self {
        Self::new(
            Register::PowerControl,
            &[(osc as u8) << 4 | (pms as u8) << 1 | (ssd as u8)],
        )
    }
    /// Defines a pulse width modulation message for setting the illuminosity of a given channel
    /// For example, setting channel 0 to 0xFF will set it to the brightest value in 8bit mode
    /// The "Update" message must be sent to see the effect of modifications to the Pwm register
    pub fn pulse_width_modulation(channel: u8, value: u16) -> Self {
        Self::new(
            Register::Pwm,
            &[(value & 0x00FF) as u8, ((value & 0xFF00) >> 8) as u8],
        )
        .register_offset(channel * 2)
    }
    /// Update all PWM registers with the loaded values
    /// Must be called after setitng the Pulse Width Modulation register
    pub fn update() -> Self {
        Self::new(Register::Update, &[0x00])
    }
    /// Adjust the global current usage of the device, see manual for detail about current usage
    pub fn global_current_control(value: u8) -> Self {
        Self::new(Register::GlobalCurrentControl, &[value])
    }
    /// Adjust individual LED current usage, see manual for detail about current usage
    pub fn led_scaling(channel: u8, value: u8) -> Self {
        Self::new(Register::LedScaling, &[value]).register_offset(channel)
    }
}

impl<I2C, EN, S> Is31fl3205<I2C, EN>
where
    I2C: Write<u8, Error = S> + Read<u8, Error = S> + WriteRead<u8, Error = S>,
    EN: OutputPin,
{
    /// Initialize the Is31fl3205 with a flexible asynchronous callback interface
    /// * `address` - User definable address which should associate with the physical AD pin(s) of the device
    /// * `en` - The enable line
    /// * `callback` - Callback for custom transmission of the address and dataframe.
    pub fn init_with_callback(address: u8, en: EN, callback: fn(addr: u8, data: &[u8])) -> Self {
        Self {
            address,
            interface: None,
            transfer_callback: Some(callback),
            enable: en,
        }
    }

    /// Initialize the Is31fl3205 with blocking i2c
    /// * `address` - User definable address which should associate with the physical AD pin(s) of the device
    /// * `en` - The enable line
    /// * `callback` - Callback for custom transmission of the address and dataframe.
    pub fn init_with_i2c(address: u8, en: EN, i2c: I2C) -> Self {
        Self {
            address,
            interface: Some(i2c),
            transfer_callback: None,
            enable: en,
        }
    }

    /// Release underlying resources
    pub fn release(self) -> (EN, Option<I2C>) {
        return (self.enable, self.interface);
    }

    /// Write a Message to the Is31fl3205, will either use the blocking i2c interface
    /// Or write to the callback for DMA / Interrupt asynchronous communication
    fn write(&mut self, message: Message) -> Result<(), Error<S>> {
        let mut buff = [0u8; 24];

        // Take the first 2 bits of the user configurable address and
        // OR it with the hardcoded manufacturer address
        let address = 0x34 | ((self.address & 0x3) << 1);

        let data = &mut buff[0..message.data_length + 1];
        data[0] = message.register as u8 + message.register_offset;
        data[1..message.data_length + 1].copy_from_slice(&message.data[0..message.data_length]);

        if self.interface.is_some() {
            self.interface
                .as_mut()
                .unwrap()
                .write(address, &data[0..message.data_length + 1])
                .map_err(Error::I2C)?;
        } else if self.transfer_callback.is_some() {
            self.transfer_callback.unwrap()(address, &data[0..message.data_length + 1]);
        } else {
            return Err(Error::InterfaceConfig);
        }
        Ok(())
    }

    /// Enable the device
    /// ub fn reset<DEL: DelayMs<u8>>(&mut self, delay: &mut DEL) -> Result<(), I2cError>
    /// Will bring the enable line low and then high, rebooting the device to a default state
    /// And then configure the device with the desired clock rate, pwm resolution and software enable / disable state
    pub fn enable_device<DEL: DelayMs<u8>>(
        &mut self,
        delay: &mut DEL,
        osc: OscillatorClock,
        pms: PwmResolution,
        ssd: SoftwareShutdownMode,
    ) -> Result<(), Error<S>> {
        // Set the enable line low to shutdown the device
        self.enable.set_low().map_err(|_| Error::EnableLine)?;
        delay.delay_ms(10 as u8);
        // Enable line must be pulled high for operation
        self.enable.set_high().map_err(|_| Error::EnableLine)?;
        self.write(Message::power_control(osc, pms, ssd))
    }

    /// Set the global current usage, a value of 0xFF will use the maximum current as allowed by the Rin resistance
    pub fn set_global_current(&mut self, value: u8) -> Result<(), Error<S>> {
        self.write(Message::global_current_control(value))
    }

    /// Set all leds to have the desired led scaling, a value of 0xFF will use the maximum current with respect to the global current
    /// configuration. See user manual for more information
    pub fn set_all_led_scaling(&mut self, value: u8) -> Result<(), Error<S>> {
        for i in 0..12 {
            self.write(Message::led_scaling(i, value))?;
        }
        Ok(())
    }
    /// Set the led to have the desired led scaling, a value of 0xFF will use the maximum current with respect to the global current
    /// configuration. See user manual for more information
    pub fn set_led_scaling(&mut self, channel: u8, value: u8) -> Result<(), Error<S>> {
        self.write(Message::led_scaling(channel, value))
    }

    /// Shutdown the device with a software shutdown and then pull the enable line low to physically turn off the device
    pub fn shutdown_device(&mut self) -> Result<(), Error<S>> {
        self.write(Message::power_control(
            OscillatorClock::FiveHundredKHz,
            PwmResolution::Eightbit,
            SoftwareShutdownMode::SoftwareShutdown,
        ))?;
        self.enable.set_low().map_err(|_| Error::EnableLine)
    }

    /// Set the desired channel value. When operating in modes less than 8bit, then only the desired number of bits will be considered
    pub fn set(&mut self, channel: u8, value: u16) -> Result<(), Error<S>> {
        self.write(Message::pulse_width_modulation(channel, value))?;
        self.write(Message::update())
    }
}
