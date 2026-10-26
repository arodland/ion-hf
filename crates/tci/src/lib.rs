//! TCI 2.0 transport for ExpertSDR: CAT and audio over one WebSocket.
//!
//! Transport only; it knows nothing about modems.

pub mod client;
pub mod protocol;

pub use client::{Config, Handler, run};
pub use protocol::{Command, Event, FrameHeader, RadioState};
