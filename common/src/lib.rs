#![no_std]
#![warn(clippy::unwrap_used)]

#[cfg(test)]
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Deserialize, Serialize)]
#[cfg_attr(test, derive(MaxSize))]
pub enum Channel {
    A,
    B,
    C,
    D,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[cfg_attr(test, derive(MaxSize))]
pub enum ToFrontend {
    Finished { channel: Channel },
}

impl Message for ToFrontend {
    const MAX_SIZE: usize = 2;
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[cfg_attr(test, derive(MaxSize))]
pub enum ToBackend {
    Start { channel: Channel },
}

impl Message for ToBackend {
    const MAX_SIZE: usize = 2;
}

pub trait Message {
    const MAX_SIZE: usize;

    fn deserialize<'a>(&self, buffer: &'a mut [u8]) -> postcard::Result<Self>
    where
        Self: Deserialize<'a>,
    {
        postcard::from_bytes_cobs(buffer)
    }

    fn serialize<'a>(&self, buffer: &'a mut [u8]) -> postcard::Result<&'a mut [u8]>
    where
        Self: Serialize,
    {
        postcard::to_slice_cobs(&self, buffer)
    }
}

#[cfg(test)]
mod tests {
    use postcard::experimental::max_size::MaxSize;

    use super::*;

    #[test]
    fn to_frontend_size() {
        assert_eq!(ToFrontend::POSTCARD_MAX_SIZE, ToFrontend::MAX_SIZE);
    }

    #[test]
    fn to_backend_size() {
        assert_eq!(ToBackend::POSTCARD_MAX_SIZE, ToBackend::MAX_SIZE);
    }
}
