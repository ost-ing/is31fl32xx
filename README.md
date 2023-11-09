# is31fl32xx
A rust-embedded driver for the Lumissil Microsystems IS31FL32xx LED driver

- https://crates.io/crates/is31fl32xx

## About 
The IS31FL3205 and IS31FL3237 are LED drivers with 12 and 36 constant current channels respectively. Each channel can be pulse width modulated (PWM) by 16 bits for smooth LED brightness control. In addition, each channel has an 8-bit output current control register which allows fine tuning the current for rich RGB color mixing, e.g., a pure white color LED application. The maximum output current of each channel can be adjusted by one 8-bit global control register.

This crate provides both blocking and DMA compatible APIs (via static callbacks) for applications utilising `embedded_hal` traits.

## Example
Examples are based on the `stm32h7xx_hal`.

```rust
// Initialize a delay provider
let mut delay: Delay = Delay::new(cp.SYST, ccdr.clocks);

// Initialize I2C pins, SCL, SDA
let scl = scl
    .into_alternate_af4()
    .internal_pull_up(true)
    .set_open_drain();
let sda = sda
    .into_alternate_af4()
    .internal_pull_up(true)
    .set_open_drain();

// Initialize the Enable line
let en = en.into_push_pull_output();

// Initialize 
let i2c: stm32h7xx_hal::i2c::I2c<I2C1> =
    i2c.i2c((scl, sda), 1_u32.MHz(), prec, clocks);

// Initialize with blocking I2C for device at address 0x00.
// The either the IS31FL3205 or IS31FL3237 models are supported
let mut interface = Is31fl32xx::<IS31FL3237, _, _>::init_with_i2c(0x00, en, i2c);

// Enable the device, bringing the enable line high and configuring the oscillator clock, pwm resolution
interface.enable_device(
    delay,
    OscillatorClock::SixteenMHz,
    PwmResolution::SixteenBit,
    SoftwareShutdownMode::Normal,
)
.unwrap();

// Set maximum current drain as specified by the current resistance
interface.set_global_current(0xFF).unwrap();

// Set brightness scaling for all leds
interface.set_all_led_scaling(0xFF).unwrap();

/// Finally set the first LED to maximum 16bit brightness. For drivers configured for less than 16bit Only the required bits are used
interface.set(0, 0xFFFF).unwrap()

// Release the blocking i2c example to regain access to its underlying resources
let (_i2c, _en) = interface.release();

// Additionally, if you need to integrate this driver with platform specific DMA controllers then
// a flexible callback can be used rather than blocking i2c
static mut DMA_BUFFER: [u8; 3] = [0; 3];
let interface = Is31fl32xx::<IS31FL3237, _, _>::init_with_callback(0x00, en, |addr, data| unsafe {
    // Copy the data from the Is31fl32xx into the DMA buffer for processing
    // NOTE: data can change in size depending on the message, the DMA implementation will
    // need to be able to handle changing slice lengths
    DMA_BUFFER[0..data.len()].copy_from_slice(data);
});
```

## Contributing
Feel free to create a ticket and a MR for any changes you would like to see in this library.
