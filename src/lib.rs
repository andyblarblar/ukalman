//! Super tiny zero allocation filters for embedded.

#![no_std]

mod kalman;
pub use kalman::Kalman1D;