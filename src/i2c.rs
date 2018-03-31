//! Inter-Integrated Circuit (I2C) bus

use cast::u8;
use stm32f103xx::{I2C1, I2C2};

use gpio::gpiob::{PB6, PB7}; // PB6 = SCLK & PB7 = SDA 
use hal::blocking::i2c::{Write, WriteRead};
use rcc::{APB1, Clocks};
use time::Hertz;
use gpio::{Alternate,PullUp};

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

const I2C_CCR_CCR : u32 = 0xFFF;

// FIXME these should be "closed" traits
/// SCL pin -- DO NOT IMPLEMENT THIS TRAIT
pub trait SclPin<I2C> {}

/// SDA pin -- DO NOT IMPLEMENT THIS TRAIT
pub trait SdaPin<I2C> {}

// unsafe impl SclPin<I2C1> for PA15<AF4> {}
impl SclPin<I2C1> for PB6<Alternate<PullUp>> {}

// unsafe impl SdaPin<I2C1> for PA14<AF4> {}
impl SdaPin<I2C1> for PB7<Alternate<PullUp>> {}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident) => {
        loop {
            //TODO
            // let isr = $i2c.isr.read();

            // if isr.berr().bit_is_set() {
            //     return Err(Error::Bus);
            // } else if isr.arlo().bit_is_set() {
            //     return Err(Error::Arbitration);
            // } else if isr.$flag().bit_is_set() {
            //     break;
            // } else {
            //     // try again
            // }
        }
    }
}

macro_rules! hal {
    ($($I2CX:ident: ($i2cX:ident, $i2cXen:ident, $i2cXrst:ident),)+) => {
        $(
            impl<SCL, SDA> I2c<$I2CX, (SCL, SDA)> {
                /// Configures the I2C peripheral to work in master mode
                pub fn $i2cX<F>(
                    i2c: $I2CX,
                    pins: (SCL, SDA),
                    freq: F,
                    clocks: Clocks,
                    apb1: &mut APB1,
                ) -> Self where
                    F: Into<Hertz>,
                    SCL: SclPin<$I2CX>,
                    SDA: SdaPin<$I2CX>,
                {
                    apb1.enr().modify(|_, w| w.$i2cXen().enabled());
                    apb1.rstr().modify(|_, w| w.$i2cXrst().set_bit());
                    apb1.rstr().modify(|_, w| w.$i2cXrst().clear_bit());

                    let freq = freq.into().0;

                    assert!(freq <= 1_000_000);

                    // TODO review compliance with the timing requirements of I2C
                    // t_I2CCLK = 1 / PCLK1
                    // t_PRESC  = (PRESC + 1) * t_I2CCLK
                    // t_SCLL   = (SCLL + 1) * t_PRESC
                    // t_SCLH   = (SCLH + 1) * t_PRESC
                    //
                    // t_SYNC1 + t_SYNC2 > 4 * t_I2CCLK
                    // t_SCL ~= t_SYNC1 + t_SYNC2 + t_SCLL + t_SCLH
                    let i2cclk = clocks.pclk1().0;
                    let freq_range = i2cclk / freq;

                    let trise = if freq <= 100_000 {
                        freq_range + 1
                    } else {
                        ((freq_range * 300) / 1000) + 1 
                    };

                    let clock_bits = if freq <= 100_000 {
                        let ccr_calc = (((i2cclk - 1 / freq) * 2) + 1) & I2C_CCR_CCR;
                        if ccr_calc < 4 {
                            4_u32
                        } else {
                            ccr_calc
                        }
                    } else { /* TODO implement higher speeds */
                        4_32
                    };


                    
                    i2c.cr2.write(|w| unsafe {
                        w.bits(freq_range)
                    });

                    /* Trise timing */
                    i2c.trise.write(|w| unsafe {
                        w.bits(trise)
                    });

                    /* Enable clocks  at given freq */
                    i2c.ccr.write(|w| unsafe {
                        w.bits(clock_bits)
                    });


                    // Enable the peripheral
                    i2c.cr1.write(|w| w.pe().set_bit());

                    I2c { i2c, pins }
                }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, (SCL, SDA)) {
                    (self.i2c, self.pins)
                }
            }

            impl<PINS> Write for I2c<$I2CX, PINS> {
                type Error = Error;

                fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);

                    /* Start condition */
                    self.i2c.cr1.write(|w| {
                        w.start()
                            .set_bit()
                    });

                    // START and prepare to send `bytes`
                    // self.i2c.cr2.write(|w| {
                    //     w.sadd1()
                    //         .bits(addr)
                    //         .rd_wrn()
                    //         .clear_bit()
                    //         .nbytes()
                    //         .bits(bytes.len() as u8)
                    //         .start()
                    //         .set_bit()
                    //         .autoend()
                    //         .set_bit()
                    // });

                    // for byte in bytes {
                    //     // Wait until we are allowed to send data (START has been ACKed or last byte
                    //     // when through)
                    //     busy_wait!(self.i2c, txis);

                    //     // put byte on the wire
                    //     self.i2c.txdr.write(|w| w.txdata().bits(*byte));
                    // }

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

                    // TODO do we have to explicitly wait here if the bus is busy (e.g. another
                    // master is communicating)?

                    // START and prepare to send `bytes`
                    // self.i2c.cr2.write(|w| {
                    //     w.sadd1()
                    //         .bits(addr)
                    //         .rd_wrn()
                    //         .clear_bit()
                    //         .nbytes()
                    //         .bits(bytes.len() as u8)
                    //         .start()
                    //         .set_bit()
                    //         .autoend()
                    //         .clear_bit()
                    // });

                    for byte in bytes {
                        // Wait until we are allowed to send data (START has been ACKed or last byte
                        // when through)
                        busy_wait!(self.i2c, txis);

                        // put byte on the wire
                        // self.i2c.txdr.write(|w| w.txdata().bits(*byte));
                    }

                    // Wait until the last transmission is finished
                    busy_wait!(self.i2c, tc);

                    // reSTART and prepare to receive bytes into `buffer`
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

                    // automatic STOP

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
