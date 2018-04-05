//! Inter-Integrated Circuit (I2C) bus

use cast::u8;
use stm32f103xx::{I2C1, I2C2};

use gpio::gpiob::{PB6, PB7, PB8, PB9, PB10, PB11};
use gpio::{Alternate, OpenDrain};
use hal::blocking::i2c::{Write, WriteRead};
use rcc::{APB1, Clocks};
use time::Hertz;

/// I2C error
#[derive(Debug)]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    // Overrun, // slave mode only
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
    #[doc(hidden)] _Extensible,
}

pub trait Pins<I2C> {
    const REMAP: bool;
}

// I2C1 - no remap
impl Pins<I2C1>
    for (
        PB6<Alternate<OpenDrain>>,
        PB7<Alternate<OpenDrain>>,
    ) {
    const REMAP: bool = false;
}

// I2C1 - remaped
impl Pins<I2C1>
    for (
        PB8<Alternate<OpenDrain>>,
        PB9<Alternate<OpenDrain>>,
    ) {
    const REMAP: bool = true;
}

// I2C2 - no remap
impl Pins<I2C2>
    for (
        PB10<Alternate<OpenDrain>>,
        PB11<Alternate<OpenDrain>>,
    ) {
    const REMAP: bool = false;
}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

// TODO impl 
// impl<PINS> I2c<SPI1, PINS> {
//     pub fn i2c1<F>(
//         spi: SPI1,
//         pins: PINS,
//         mapr: &mut MAPR,
//         mode: Mode,
//         freq: F,
//         clocks: Clocks,
//         apb: &mut APB2,
//     ) -> Self
//     where
//         F: Into<Hertz>,
//         PINS: Pins<SPI1>,
//     {
//         mapr.mapr().modify(|_, w| w.spi1_remap().bit(PINS::REMAP));
//         Spi::_spi1(spi, pins, mode, freq.into(), clocks, apb)
//     }
// }

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident) => {
        loop {
            let isr = $i2c.sr1.read();

            if isr.berr().bit_is_set() {
                return Err(Error::Bus);
            } else if isr.arlo().bit_is_set() {
                return Err(Error::Arbitration);
            } else if isr.$flag().bit_is_set() {
                break;
            } else {
                // try again
            }
        }
    }
}

macro_rules! hal {
    ($($I2CX:ident: ($i2cX:ident, $i2cXen:ident, $i2cXrst:ident),)+) => {
        $(
            impl<PINS> I2c<$I2CX, PINS> {
                /// Configures the I2C peripheral to work in master mode
                pub fn $i2cX<F>(
                    i2c: $I2CX,
                    pins: PINS,
                    freq: F,
                    clocks: Clocks,
                    apb1: &mut APB1,
                ) -> Self where
                    F: Into<Hertz>,
                    // PINS: Pins<$I2CX> //TODO impl enforcement on pins
                {
                    apb1.enr().modify(|_, w| w.$i2cXen().enabled());
                    apb1.rstr().modify(|_, w| w.$i2cXrst().set_bit());
                    apb1.rstr().modify(|_, w| w.$i2cXrst().clear_bit());

                    let clock_speed = freq.into().0;

                    assert!(clock_speed <= 1_000_000);

                    let i2cclk = clocks.pclk1().0;
                    let freq_range = i2cclk / 1_000_000;
                    assert!(freq_range >= 2);
                    assert!(freq_range <= 50);

                    // (((clock_speed) <= 100000U) ? ((__FREQRANGE__) + 1U) : ((((__FREQRANGE__) * 300U) / 1000U) + 1U))
                    let trise = if clock_speed <= 100_000 {
                        freq_range + 1
                    } else {
                        ((freq_range * 300) / 1000) + 1
                    };

                    const CCR_COEFF: u32 = 2;
                    const CCR_MASK: u32 = 0x0FFF;
                    let ccr = if clock_speed <= 100_000 {
                        // I2C_SPEED_STANDARD
                        // I2C_CCR_CALCULATION((__PCLK__), (__SPEED__), 2U) < 4U) ? 4U : I2C_CCR_CALCULATION((__PCLK__), (__SPEED__), 2U)
                        let ccr_calc =  (i2cclk - 1) / (((clock_speed * CCR_COEFF) + 1) & CCR_MASK);
                        if ccr_calc < 4 {
                            4
                        } else {
                            ccr_calc
                        }
                    } else {
                        // TODO impl SPEED_FAST
                        // (((__DUTYCYCLE__) == I2C_DUTYCYCLE_2)? I2C_CCR_CALCULATION((__PCLK__), (__SPEED__), 3U) : (I2C_CCR_CALCULATION((__PCLK__), (__SPEED__), 25U) | I2C_DUTYCYCLE_16_9))
                        4
                    };
                    /* Tell peripheral is bus speed so it can generate correct clocks */
                    i2c.cr2.modify(|_, w| unsafe {
                        w.bits(freq_range)
                    });
                    /* Setup rise time */
                    i2c.trise.modify(|_, w| unsafe {
                        w.bits(trise)
                    });
                    /* Setup clocks */
                    i2c.ccr.modify(|_, w| unsafe {
                        w.bits(ccr)
                    });

                    // Enable the peripheral
                    i2c.cr1.write(|w| w.pe().set_bit());

                    I2c { i2c, pins }
                }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, PINS) {
                    (self.i2c, self.pins)
                }
            }

            impl<PINS> Write for I2c<$I2CX, PINS> {
                type Error = Error;

                fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);

                    // START and prepare to send `bytes`
                    self.i2c.cr1.write(|w| {
                        w.start().set_bit()
                    });

                    while !self.i2c.sr1.read().sb().bit_is_set() {} // wait for start byte to be sent
                    
                    self.i2c.dr.write(|w| unsafe { // write the slave address on the line
                        w.bits(addr as u32)
                    });

                    while !self.i2c.sr1.read().addr().bit_is_set() {} // wait for addr byte to be sent
                    self.i2c.sr2.read(); // peripher expects an sr2 read

                    for byte in bytes {
                        // Wait until we are allowed to send data (START has been ACKed or last byte
                        // when through)
                        // put byte on the wire

                        self.i2c.dr.write(|w| unsafe { w.bits(*byte as u32) } );

                        busy_wait!(self.i2c, tx_e);

                    }

                    // Wait until the last transmission is finished ???
                    // busy_wait!(self.i2c, busy);

                    // automatic STOP

                    Ok(())
                }
            }

            impl<PINS> WriteRead for I2c<$I2CX, PINS> {
                type Error = Error;

                fn write_read(
                    &mut self,
                    addr: u8,
                    bytes: &[u8],
                    buffer: &mut [u8],
                ) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);
                    assert!(buffer.len() < 256 && buffer.len() > 0);

                    // // TODO do we have to explicitly wait here if the bus is busy (e.g. another
                    // // master is communicating)?

                    // // START and prepare to send `bytes`
                    // self.i2c.cr2.write(|w| {
                    //     w.start().set_bit();
                    // });

                    // for byte in bytes {
                    //     // Wait until we are allowed to send data (START has been ACKed or last byte
                    //     // when through)
                    //     busy_wait!(self.i2c, txis);

                    //     // put byte on the wire
                    //     self.i2c.txdr.write(|w| w.txdata().bits(*byte));
                    // }

                    // // Wait until the last transmission is finished
                    // busy_wait!(self.i2c, tc);

                    // // reSTART and prepare to receive bytes into `buffer`
                    // self.i2c.cr2.write(|w| {
                    //     w.sadd1()
                    //         .bits(addr)
                    //         .rd_wrn()
                    //         .set_bit()
                    //         .nbytes()
                    //         .bits(buffer.len() as u8)
                    //         .start()
                    //         .set_bit()
                    //         .autoend()
                    //         .set_bit()
                    // });

                    // for byte in buffer {
                    //     // Wait until we have received something
                    //     busy_wait!(self.i2c, rxne);

                    //     *byte = self.i2c.rxdr.read().rxdata().bits();
                    // }

                    // // automatic STOP

                    Ok(())
                }
            }
        )+
    }
}

hal! {
    I2C1: (i2c1, i2c1en, i2c1rst),
    I2C2: (i2c2, i2c2en, i2c2rst),
}