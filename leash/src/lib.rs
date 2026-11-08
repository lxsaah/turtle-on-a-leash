//! The rules that hold the turtle back.
//!
//! Pure logic with no AimDB or board types: [`Leash`] takes events and a clock
//! and emits effects, so every race the demo cares about can be tested on the
//! host. The firmware only translates between records and these calls.
#![no_std]
extern crate alloc;

pub mod controller;
pub mod geometry;
pub mod guard;
mod machine;
pub mod rules;

pub use machine::{Effect, Event, Leash};
