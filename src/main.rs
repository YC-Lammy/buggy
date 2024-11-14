#![no_std]
#![no_main]

/// implements an async timer
mod timer;

use arduino_hal::{default_serial, simple_pwm::*};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::pwm::SetDutyCycle;
use panic_halt as _;
use timer::Timer;

pub struct Motor<H: SetDutyCycle, L: SetDutyCycle> {
    pwm0: H,
    pwm1: L,
    /// strength in -100 to 100
    strength: i8,
}

impl<H: SetDutyCycle, L: SetDutyCycle> Motor<H, L> {
    pub fn new(h: H, l: L) -> Self {
        Self {
            pwm0: h,
            pwm1: l,
            strength: 0,
        }
    }

    pub fn disable(&mut self) {
        self.pwm0.set_duty_cycle(0).ok();
        self.pwm1.set_duty_cycle(0).ok();
    }

    pub fn set_strength_percentage(&mut self, p: i8) {
        self.strength = p;
        if self.strength < 0 {
            self.pwm0.set_duty_cycle_percent(p.abs() as u8).ok();
            self.pwm1.set_duty_cycle(0).ok();
        } else {
            self.pwm0.set_duty_cycle(0).ok();
            self.pwm1.set_duty_cycle_percent(p as u8).ok();
        }
    }
}

pub async fn turn_right<P0, P1, P2, P3>(
    timer: &Timer,
    left: &mut Motor<P0, P1>,
    right: &mut Motor<P2, P3>,
) where
    P0: SetDutyCycle,
    P1: SetDutyCycle,
    P2: SetDutyCycle,
    P3: SetDutyCycle,
{
    left.set_strength_percentage(100);
    right.set_strength_percentage(-100);

    timer.after_millis(500).await;

    left.set_strength_percentage(0);
    right.set_strength_percentage(0);
}

pub async fn turn_left<P0, P1, P2, P3>(
    timer: &Timer,
    left: &mut Motor<P0, P1>,
    right: &mut Motor<P2, P3>,
) where
    P0: SetDutyCycle,
    P1: SetDutyCycle,
    P2: SetDutyCycle,
    P3: SetDutyCycle,
{
    left.set_strength_percentage(-100);
    right.set_strength_percentage(100);

    timer.after_millis(500).await;

    left.set_strength_percentage(0);
    right.set_strength_percentage(0);
}

#[derive(Debug, Clone, Copy)]
pub enum ServoDirection {
    Centre,
    Left,
    Right,
}

pub struct ServoMotor<'a, P: OutputPin> {
    pin: P,
    timer: &'a Timer,
    direction: ServoDirection,
}

impl<'a, P> ServoMotor<'a, P>
where
    P: OutputPin,
{
    pub fn new(timer: &'a Timer, pin: P) -> Self {
        Self {
            pin,
            timer,
            direction: ServoDirection::Centre,
        }
    }

    fn cast_mut(&self) -> &mut Self {
        unsafe { (self as *const Self as *mut Self).as_mut().unwrap() }
    }

    /// pulse task contineously generates pulses at 50hz
    /// duty cycle is determined by the servo direction
    pub async fn pulse_task(&self) {
        let pin = unsafe { (&self.pin as *const P as *mut P).as_mut().unwrap() };

        loop {
            // a pulse is one cycle of wave
            let pulse = self.timer.after_millis(20);

            let us = match self.direction {
                ServoDirection::Left => 1000,
                ServoDirection::Centre => 1500,
                ServoDirection::Right => 2000,
            };

            // set pin to high
            pin.set_high().ok();
            // wait for 2.8ms
            self.timer.after_micros(us).await;
            // set pin to low
            pin.set_low().ok();

            // wait for the 20ms to finish
            pulse.await;
        }
    }

    pub fn turn_right(&self) {
        self.cast_mut().direction = ServoDirection::Right
    }

    pub fn turn_left(&self) {
        self.cast_mut().direction = ServoDirection::Left
    }

    pub fn turn_centre(&self) {
        self.cast_mut().direction = ServoDirection::Centre
    }
}

pub struct HCSR04<T, E>
where
    T: OutputPin,
    E: InputPin,
{
    trig: T,
    echo: E,
}

impl<T, E> HCSR04<T, E>
where
    T: OutputPin,
    E: InputPin,
{
    pub fn new(trig: T, echo: E) -> Self {
        Self { trig, echo }
    }

    /// trigger and return distance in cm
    pub async fn detect(&mut self, timer: &Timer) -> Option<u32> {
        // set trigger high to trigger
        let _ = self.trig.set_high();
        // wait for 12 us
        timer.after_micros(12).await;
        // finish trigger
        let _ = self.trig.set_low();

        let now = timer.now();

        while self.echo.is_low().unwrap() {
            // nothing is detected if 200 ms has passed
            if timer.micros_since(&now) >= 200_000 {
                return None;
            }
        }

        let now = timer.now();

        while self.echo.is_high().unwrap() {
            // do nothing
        }

        let micros = timer.micros_since(&now);

        return Some(micros / 58);
    }
}

pub async fn main_task<T, E, S, P0, P1, P2, P3>(
    mut serial: impl ufmt::uWrite,
    timer: &Timer,
    mut hcsr04: HCSR04<T, E>,
    servo: &ServoMotor<'_, S>,
    mut left: Motor<P0, P1>,
    mut right: Motor<P2, P3>,
) -> !
where
    T: OutputPin,
    E: InputPin,
    S: OutputPin,
    P0: SetDutyCycle,
    P1: SetDutyCycle,
    P2: SetDutyCycle,
    P3: SetDutyCycle,
{
    const SAMPLES: u32 = 2;

    let mut samples = 0;
    let mut values = 0;

    let mut needs_look_around = false;
    let mut is_backward = false;

    loop {
        if !needs_look_around {
            if let Some(v) = hcsr04.detect(&timer).await {
                // filter out 0
                if v != 0 {
                    samples += 1;
                    values += v;
                }
            }

            if samples == SAMPLES {
                let distance = values / SAMPLES;

                samples = 0;
                values = 0;

                // hault the motors if somthing is in front
                if distance < 7 {
                    needs_look_around = true;
                    left.set_strength_percentage(0);
                    right.set_strength_percentage(0);
                }

                let _ = ufmt::uwriteln!(&mut serial, "detect distance: {} cm", distance);
            }
        }

        if needs_look_around {
            servo.turn_left();
            timer.after_millis(40).await;

            let left_distance = hcsr04.detect(timer).await.unwrap_or(u32::MAX);

            servo.turn_right();
            timer.after_millis(40).await;

            let right_distance = hcsr04.detect(timer).await.unwrap_or(u32::MAX);

            servo.turn_centre();

            let _ = ufmt::uwriteln!(
                &mut serial,
                "left: {} cm, right: {} cm",
                left_distance,
                right_distance
            );

            // go backeards if blocks
            if left_distance < 7 && right_distance < 7 {
                // still need to look around after going backwards
                needs_look_around = true;
                is_backward = true;

                left.set_strength_percentage(-10);
                right.set_strength_percentage(-10);
            } else if left_distance >= right_distance {
                // turn left
                turn_left(timer, &mut left, &mut right).await;

                left.set_strength_percentage(50);
                right.set_strength_percentage(50);

                needs_look_around = false;
            } else {
                // turn right
                turn_right(timer, &mut left, &mut right).await;

                left.set_strength_percentage(50);
                right.set_strength_percentage(50);

                needs_look_around = false;
            }
        }
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let serial = default_serial!(dp, pins, 57600);

    let timer0 = Timer0Pwm::new(dp.TC0, Prescaler::Prescale64);
    let timer2 = Timer2Pwm::new(dp.TC2, Prescaler::Prescale64);

    // Digital pin 5 is connected to a LED and a resistor in series
    let mut pwm0 = pins.d5.into_output().into_pwm(&timer0);
    let mut pwm1 = pins.d6.into_output().into_pwm(&timer0);
    let mut pwm2 = pins.d3.into_output().into_pwm(&timer2);
    let mut pwm3 = pins.d11.into_output().into_pwm(&timer2);

    pwm0.set_duty_cycle_percent(0).ok();
    pwm1.set_duty_cycle_percent(0).ok();

    pwm2.set_duty_cycle_percent(0).ok();
    pwm3.set_duty_cycle_percent(0).ok();

    pwm0.enable();
    pwm1.enable();
    pwm2.enable();
    pwm3.enable();

    let motor_left = Motor::new(pwm0, pwm1);
    let motor_right = Motor::new(pwm2, pwm3);

    let trig = pins.d12.into_output();
    let echo = pins.d13; // pin is input by default

    let hcsr04 = HCSR04::new(trig, echo);

    let timer = Timer::new(dp.TC1);

    // Important because this sets the bit in the DDR register!
    let d9 = pins.d9.into_output();

    let servo = ServoMotor::new(&timer, d9);

    let pulse_task = servo.pulse_task();
    let main_task = main_task(serial, &timer, hcsr04, &servo, motor_left, motor_right);

    // run pulse task and main task concurrently
    embassy_futures::block_on(embassy_futures::join::join(pulse_task, main_task));

    unreachable!()
}
