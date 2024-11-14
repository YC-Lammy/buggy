use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use arduino_hal::pac::TC1;

pub struct Timer {
    /// the timer 1 peripheral
    tc1: TC1,
    /// each tick is 4 µs
    ///
    /// 64 bits gives us 146,235.6 years
    ticks: u64,
    /// number of objects dependant on the timer
    queue: u8,
}

impl Timer {
    pub fn new(tc1: TC1) -> Self {
        // Starting and initializing the timer with prescaling 64.
        // it gives one clock count every 4 µs.
        // since the clock register size is 16 bits, the timer is full every
        // 1/(16e6/64)*2^16 ≈ 260 ms
        tc1.tccr1b.write(|w| w.cs1().prescale_64());
        // the timer is reinitialized with value 0.
        tc1.tcnt1.write(|w| w.bits(0));

        Self {
            tc1,
            ticks: 0,
            queue: 0,
        }
    }

    /// a hack to get rid of mutable reference
    fn cast_mut(&self) -> &mut Self {
        unsafe { (self as *const Self as *mut Self).as_mut().unwrap() }
    }

    // update the number of ticks
    fn update_ticks(&mut self) {
        // if the queue is 0, reinitialise timer
        if self.queue == 0 {
            // the timer is reinitialized with value 0.
            self.tc1.tcnt1.write(|w| w.bits(0));
            self.ticks = 0;

            return;
        }

        // read number of ticks
        let t = self.tc1.tcnt1.read().bits();

        // the timer is reinitialized with value 0.
        self.tc1.tcnt1.write(|w| w.bits(0));

        // add the ticks to total ticks
        self.ticks += t as u64;
    }

    pub fn after_millis(&self, millis: u64) -> TimerAfter {
        self.after_micros(millis * 1000)
    }

    pub fn after_micros(&self, micros: u64) -> TimerAfter {
        // get mutable reference to self
        let t = self.cast_mut();
        // update number of tick
        t.update_ticks();
        // add to queue
        t.queue += 1;

        // get the current number of ticks
        let current = self.ticks;

        // caculate the duration in ticks
        let duration_ticks = micros / 4;

        // return the future
        return TimerAfter {
            timer: self,
            target: current + duration_ticks,
        };
    }

    pub fn now(&self) -> Instant {
        let t = self.cast_mut();
        t.update_ticks();
        t.queue += 1;

        Instant {
            timer: self,
            ticks: self.ticks,
        }
    }

    pub fn micros_since(&self, instant: &Instant) -> u32 {
        let t = self.cast_mut();
        t.update_ticks();

        let ticks = self.ticks;

        return (ticks - instant.ticks) as u32 * 4;
    }
}

/// a structure that stores the target timer ticks
pub struct TimerAfter<'a> {
    timer: &'a Timer,
    target: u64,
}

impl<'a> Drop for TimerAfter<'a> {
    fn drop(&mut self) {
        let timer = self.timer.cast_mut();
        timer.queue -= 1;
        timer.update_ticks();
    }
}

/// implement future for timer wait for async operation
impl<'a> Future for TimerAfter<'a> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<Self::Output> {
        self.timer.cast_mut().update_ticks();

        if self.timer.ticks >= self.target {
            return Poll::Ready(());
        }

        return Poll::Pending;
    }
}

pub struct Instant<'a> {
    timer: &'a Timer,
    ticks: u64,
}

impl<'a> Drop for Instant<'a> {
    fn drop(&mut self) {
        let timer = self.timer.cast_mut();
        timer.queue -= 1;
        timer.update_ticks();
    }
}
