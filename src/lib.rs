#![no_std]
#![allow(dead_code)]

mod constants;
mod cursor;
mod descriptors;

use constants::*;
use descriptors::*;

use usb_device::control::{Recipient, Request, RequestType};
use usb_device::device::DEFAULT_ALTERNATE_SETTING;
use usb_device::endpoint::{self, Endpoint, EndpointDirection, In, Out};
use usb_device::{UsbDirection, class_prelude::*};

#[derive(Clone, Copy, Debug)]
pub enum Format {
    /// Signed, 16 bits per subframe, little endian
    S16le,
    /// Signed, 24 bits per subframe, little endian
    S24le,
    /// Signed, 32 bits per subframe, little endian
    S32le,
}

/// Sampling rates that shall be supported by an steaming endpoint
#[derive(Debug)]
pub enum Rates<'a> {
    /// A continuous range of sampling rates in samples/second defined by a
    /// tuple including a minimum value and a maximum value. The maximum value
    /// must be greater than the minimum value.
    Continuous(u32, u32),
    /// A set of discrete sampling rates in samples/second
    Discrete(&'a [u32]),
}

#[derive(Debug)]
pub struct StreamConfig<'a> {
    format: Format,
    channels: u8,
    rates: Rates<'a>,
    terminal_type: TerminalType,
    /// ISO endpoint size calculated from format, channels and rates (may be
    /// removed in future)
    ep_size: u16,
}

impl StreamConfig<'_> {
    /// Create a stream configuration with one or more discrete sampling rates
    /// indicated in samples/second. An input stream or an output stream will
    /// have an Input Terminal or Output Terminal of Terminal Type
    /// `terminal_type`, respectively.
    pub fn new_discrete(
        format: Format,
        channels: u8,
        rates: &'_ [u32],
        terminal_type: TerminalType,
    ) -> Result<StreamConfig<'_>> {
        let max_rate = rates.iter().max().unwrap();
        let ep_size = Self::ep_size(format, channels, *max_rate)?;
        let rates = Rates::Discrete(rates);
        Ok(StreamConfig {
            format,
            channels,
            rates,
            terminal_type,
            ep_size,
        })
    }

    /// Create a stream configuration with a continuous range of supported
    /// sampling rates indicated in samples/second. An input stream or an output
    /// stream will have an Input Terminal or Output Terminal of Terminal Type
    /// `terminal_type`, respectively.
    pub fn new_continuous(
        format: Format,
        channels: u8,
        min_rate: u32,
        max_rate: u32,
        terminal_type: TerminalType,
    ) -> Result<StreamConfig<'static>> {
        if min_rate >= max_rate {
            return Err(Error::InvalidValue);
        }
        let ep_size = Self::ep_size(format, channels, max_rate)?;
        let rates = Rates::Continuous(min_rate, max_rate);
        Ok(StreamConfig {
            format,
            channels,
            rates,
            terminal_type,
            ep_size,
        })
    }

    /// calculate ISO endpoint size from format, channels and rates
    fn ep_size(format: Format, channels: u8, max_rate: u32) -> Result<u16> {
        let octets_per_frame = channels as u32
            * match format {
                Format::S16le => 2,
                Format::S24le => 3,
                Format::S32le => 4,
            };
        let ep_size = octets_per_frame * max_rate / 1000;
        // if ep_size > MAX_ISO_EP_SIZE {
        //     return Err(Error::BandwidthExceeded);
        // }
        Ok(ep_size as u16)
    }
}

/// USB audio errors, including possible USB Stack errors
#[derive(Debug)]
pub enum Error {
    InvalidValue,
    BandwidthExceeded,
    StreamNotInitialized,
    UsbError(usb_device::UsbError),
}

impl From<UsbError> for Error {
    fn from(err: UsbError) -> Self {
        Error::UsbError(err)
    }
}
type Result<T> = core::result::Result<T, Error>;

struct AudioStream<'a, B: UsbBus, D: EndpointDirection> {
    stream_config: StreamConfig<'a>,
    interface: InterfaceNumber,
    endpoint: Endpoint<'a, B, D>,
    alt_setting: u8,
}

impl<B: UsbBus> AudioStream<'_, B, endpoint::In> {
    fn input_terminal_desc(&self, id: u8, clock_source: u8) -> InputTerminal {
        let channel_config = ChannelConfig::default_chans(self.stream_config.channels);
        InputTerminal {
            id,
            terminal_type: TerminalType::UsbStreaming,
            assoc_terminal: 0,
            clock_source,
            num_channels: self.stream_config.channels,
            channel_config,
            channel_names: 0,
            copy_protect_control: AccessControl::NotPresent,
            connector_control: AccessControl::NotPresent,
            overload_control: AccessControl::NotPresent,
            cluster_control: AccessControl::NotPresent,
            underflow_control: AccessControl::NotPresent,
            overflow_control: AccessControl::NotPresent,
            phantom_power_control: AccessControl::NotPresent,
            string: 0,
        }
    }
}

impl<B: UsbBus> AudioStream<'_, B, endpoint::Out> {
    fn output_terminal_desc(&self, id: u8, source_id: u8, clock_source: u8) -> OutputTerminal {
        OutputTerminal {
            id,
            terminal_type: TerminalType::UsbStreaming,
            assoc_terminal: 0,
            source_id,
            clock_source,
            copy_protect_control: AccessControl::NotPresent,
            connector_control: AccessControl::NotPresent,
            overload_control: AccessControl::NotPresent,
            underflow_control: AccessControl::NotPresent,
            overflow_control: AccessControl::NotPresent,
            string: 0,
        }
    }
}

pub struct AudioClass<'a, B: UsbBus> {
    control_iface: InterfaceNumber,
    input: Option<AudioStream<'a, B, In>>,
    output: Option<AudioStream<'a, B, Out>>,
    function: FunctionCode,
    clock_type: ClockType,
    input_type: Option<TerminalType>,
    output_type: Option<TerminalType>,
}

impl<B: UsbBus> AudioClass<'_, B> {}

impl<B: UsbBus> UsbClass<B> for AudioClass<'_, B> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut DescriptorWriter,
    ) -> usb_device::Result<()> {
        // Build the necessary descriptors
        //  Clock Source - id 1
        //  USB Input Terminal - id 2
        //  Audio Output Terminal - id 3
        //  USB Output Terminal - id 4
        //  Audio Input Terminal - id 5
        let clock_source = ClockSource {
            id: 1,
            clock_type: self.clock_type,
            sof_sync: false,
            frequency_access: if self.clock_type == ClockType::InternalProgrammable {
                AccessControl::Programmable
            } else {
                AccessControl::NotPresent
            },
            validity_access: AccessControl::ReadOnly,
            assoc_terminal: 0,
            string: 0,
        };
        let in_terminals = match &self.input {
            Some(i) => Some((
                i.input_terminal_desc(2, 1),
                OutputTerminal {
                    id: 3,
                    terminal_type: self.output_type.unwrap_or(TerminalType::OutUndefined),
                    assoc_terminal: 0,
                    source_id: 2,
                    clock_source: 1,
                    copy_protect_control: AccessControl::NotPresent,
                    connector_control: AccessControl::NotPresent,
                    overload_control: AccessControl::NotPresent,
                    underflow_control: AccessControl::NotPresent,
                    overflow_control: AccessControl::NotPresent,
                    string: 0,
                },
            )),
            None => None,
        };
        let out_terminals = match &self.output {
            Some(i) => Some((
                i.output_terminal_desc(4, 5, 1),
                InputTerminal {
                    id: 5,
                    terminal_type: self.input_type.unwrap_or(TerminalType::InUndefined),
                    assoc_terminal: 0,
                    clock_source: 1,
                    num_channels: i.stream_config.channels,
                    channel_config: ChannelConfig::default_chans(i.stream_config.channels),
                    channel_names: 0,
                    copy_protect_control: AccessControl::NotPresent,
                    connector_control: AccessControl::NotPresent,
                    overload_control: AccessControl::NotPresent,
                    cluster_control: AccessControl::NotPresent,
                    underflow_control: AccessControl::NotPresent,
                    overflow_control: AccessControl::NotPresent,
                    phantom_power_control: AccessControl::NotPresent,
                    string: 0,
                },
            )),
            None => None,
        };
        let n_interfaces = match (&self.input, &self.output) {
            (Some(_), Some(_)) => 3,                // two audio, one control
            (Some(_), None) | (None, Some(_)) => 2, // one audio, one control
            (None, None) => 1,                      // no audio (?!), one control
        };
        writer.iad(
            self.control_iface,
            n_interfaces,
            AUDIO,
            FunctionSubclass::Undefined as u8,
            FunctionProtocol::Version2 as u8,
            None,
        )?;
        writer.interface(
            self.control_iface,
            AUDIO,
            InterfaceSubclass::AudioControl as u8,
            InterfaceProtocol::Version2 as u8,
        )?;

        if let Some(terminals) = in_terminals {
            terminals.0.write_descriptor(writer)?;
            terminals.1.write_descriptor(writer)?;
        }
        if let Some(terminals) = out_terminals {
            terminals.0.write_descriptor(writer)?;
            terminals.1.write_descriptor(writer)?;
        }

        Ok(())
    }
}
