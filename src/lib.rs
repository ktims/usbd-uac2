#![no_std]
#![allow(dead_code)]

mod constants;
mod cursor;
mod descriptors;

use core::marker::PhantomData;

use constants::*;
use descriptors::*;

use usb_device::control::{Recipient, Request, RequestType};
use usb_device::device::DEFAULT_ALTERNATE_SETTING;
use usb_device::endpoint::{self, Endpoint, EndpointDirection, In, Out};
use usb_device::{UsbDirection, class_prelude::*};

mod sealed {
    pub trait Sealed {}
}

pub enum UsbAudioClassError {
    NotImplemented,
    Other,
}

impl<T: core::error::Error> From<T> for UsbAudioClassError {
    fn from(_: T) -> Self {
        UsbAudioClassError::Other
    }
}

pub trait RangeType: sealed::Sealed {}
impl sealed::Sealed for i8 {}
impl sealed::Sealed for i16 {}
impl sealed::Sealed for i32 {}
impl sealed::Sealed for u8 {}
impl sealed::Sealed for u16 {}
impl sealed::Sealed for u32 {}

impl RangeType for i8 {}
impl RangeType for i16 {}
impl RangeType for i32 {}
impl RangeType for u8 {}
impl RangeType for u16 {}
impl RangeType for u32 {}

pub struct RangeEntry<T: RangeType> {
    min: T,
    max: T,
    res: T,
}

impl<T: RangeType> RangeEntry<T> {
    pub fn new(min: T, max: T, res: T) -> Self {
        Self { min, max, res }
    }
}

/// A trait for implementing USB Audio Class 2 devices
///
/// Contains optional callback methods which will be called by the class driver. All
/// callbacks are optional, which may be useful for a tight-loop polling implementation
/// but most implementations will want to implement at least `audio_data_rx`.
///
/// Unimplemented callbacks should return `Err(UsbAudioClassError::NotImplemented)`. Other
/// errors will panic (the underlying callbacks are not fallible). If you need to handle errors,
/// you should use the callback to infalliably signal another task.
pub trait UsbAudioClass<'a, B: UsbBus> {
    /// Called when audio data is received from the host. The `Endpoint`
    /// is ready for `read()`.
    fn audio_data_rx(
        &self,
        ep: &Endpoint<'a, B, endpoint::Out>,
    ) -> core::result::Result<(), UsbAudioClassError> {
        Err(UsbAudioClassError::NotImplemented)
    }
}

/// A trait for implementing Sampling Frequency Control for USB Audio Clock Sources
/// ref: USB Audio Class Specification 2.0 5.2.5.1.1
///
/// Contains optional callback methods which will be called by the class driver. If
/// `set_sample_rate` is implemented, `get_sample_rate` must also be implemented.
/// Callbacks run in USB context, so should not block.
///
/// Unimplemented callbacks should return `Err(UsbAudioClassError::NotImplemented)`. Other
/// errors will panic (the underlying callbacks are not fallible). If you need to handle errors,
/// you should use the callback to infalliably signal another task.
pub trait UsbAudioClockSource {
    const CLOCK_TYPE: ClockType;
    const SOF_SYNC: bool;
    /// Called when the host requests the current sample rate. Returns the sample rate in Hz.
    fn get_sample_rate(&self) -> core::result::Result<u32, UsbAudioClassError> {
        Err(UsbAudioClassError::NotImplemented)
    }
    /// Called when the host requests to set the sample rate. Should reconfigure the clock source
    /// if necessary.
    fn set_sample_rate(
        &mut self,
        sample_rate: u32,
    ) -> core::result::Result<(), UsbAudioClassError> {
        Err(UsbAudioClassError::NotImplemented)
    }
    /// Called when the host requests to get the clock validity. Returns `true`
    /// if the clock is stable and on frequency.
    fn get_clock_validity(&self) -> core::result::Result<bool, UsbAudioClassError> {
        Err(UsbAudioClassError::NotImplemented)
    }
    /// Called during descriptor construction to describe if the clock validity can be read (write is not valid).
    ///
    /// By default will call `get_clock_validity` to determine if the clock validity can be read.
    fn get_validity_access(&self) -> core::result::Result<bool, UsbAudioClassError> {
        match self.get_clock_validity() {
            Ok(_) => Ok(true),
            Err(UsbAudioClassError::NotImplemented) => Ok(false),
            Err(err) => Err(err),
        }
    }

    /// Called when the hosts makes a RANGE request for the clock source. Returns a slice of possible sample rates.
    ///
    /// Must be implemented if the clock source returns programmable get_frequency_access
    ///
    /// Rates must meet the invariants in the specification:
    ///   * The subranges must be ordered in ascendingorder
    ///   * Individual subranges cannot overlap
    ///   * If a subrange consists of only a single value, the corresponding triplet must contain that value for both
    ///     its MIN and MAX subattribute and the RES subattribute must be set to zero
    ///
    /// ref: USB Audio Class Specification 2.0 5.2.1 & 5.2.3.3
    fn get_rates(&self) -> core::result::Result<&[RangeEntry<u32>], UsbAudioClassError> {
        Err(UsbAudioClassError::NotImplemented)
    }

    /// Build the ClockSource descriptor. It is not intended to override this method.
    ///
    /// Assumes access control based on clock type. Internal fixed/variable are read only,
    /// external and internal programmable are programmable.
    fn get_configuration_descriptor(
        &self,
        id: u8,
        string: Option<StringIndex>,
    ) -> usb_device::Result<ClockSource> {
        let frequency_access = match Self::CLOCK_TYPE {
            ClockType::InternalFixed | ClockType::InternalVariable => AccessControl::ReadOnly,
            ClockType::External | ClockType::InternalProgrammable => AccessControl::Programmable,
        };
        let validity_access = match self.get_validity_access() {
            Ok(true) => AccessControl::ReadOnly,
            Ok(false) | Err(UsbAudioClassError::NotImplemented) => AccessControl::NotPresent,
            _ => return Err(UsbError::Unsupported),
        };

        let cs = ClockSource {
            id: id,
            clock_type: Self::CLOCK_TYPE,
            sof_sync: Self::SOF_SYNC,
            frequency_access,
            validity_access,
            assoc_terminal: 0,
            string,
        };
        Ok(cs)
    }
}

pub struct TerminalConfig<D: EndpointDirection> {
    clock_source_id: u8,
    num_channels: u8,
    format: FormatType1,
    terminal_type: TerminalType,
    channel_config: ChannelConfig,
    string: Option<StringIndex>,
    _direction: PhantomData<D>,
}

impl<'a, D: EndpointDirection> TerminalConfig<D> {
    pub fn new(
        clock_source_id: u8,
        num_channels: u8,
        format: FormatType1,
        terminal_type: TerminalType,
        channel_config: ChannelConfig,
        string: Option<StringIndex>,
    ) -> Self {
        TerminalConfig {
            clock_source_id,
            num_channels,
            format,
            terminal_type,
            channel_config,
            string,
            _direction: PhantomData,
        }
    }
}
impl<'a> TerminalConfig<In> {
    fn get_configuration_descriptors(&self, start_id: u8) -> (InputTerminal, OutputTerminal) {
        let input_terminal = InputTerminal {
            id: start_id,
            terminal_type: TerminalType::UsbStreaming,
            assoc_terminal: start_id + 1,
            clock_source: self.clock_source_id,
            num_channels: self.num_channels,
            channel_config: self.channel_config,
            channel_names: 0, // not supported
            copy_protect_control: AccessControl::NotPresent,
            connector_control: AccessControl::NotPresent,
            overload_control: AccessControl::NotPresent,
            cluster_control: AccessControl::NotPresent,
            underflow_control: AccessControl::NotPresent,
            overflow_control: AccessControl::NotPresent,
            phantom_power_control: AccessControl::NotPresent,
            string: None,
        };
        let output_terminal = OutputTerminal {
            id: start_id + 1,
            terminal_type: self.terminal_type,
            assoc_terminal: start_id,
            source_id: start_id,
            clock_source: self.clock_source_id,
            copy_protect_control: AccessControl::NotPresent,
            connector_control: AccessControl::NotPresent,
            overload_control: AccessControl::NotPresent,
            underflow_control: AccessControl::NotPresent,
            overflow_control: AccessControl::NotPresent,
            string: self.string,
        };
        (input_terminal, output_terminal)
    }
    // fn get_interface_descriptor(&self, id: InterfaceIndex) )
}

impl<'a> TerminalConfig<Out> {
    fn get_configuration_descriptors(&self, start_id: u8) -> (OutputTerminal, InputTerminal) {
        let output_terminal = OutputTerminal {
            id: start_id,
            terminal_type: TerminalType::UsbStreaming,
            assoc_terminal: start_id + 1,
            source_id: start_id + 1,
            clock_source: self.clock_source_id,
            copy_protect_control: AccessControl::NotPresent,
            connector_control: AccessControl::NotPresent,
            overload_control: AccessControl::NotPresent,
            underflow_control: AccessControl::NotPresent,
            overflow_control: AccessControl::NotPresent,
            string: self.string,
        };
        let input_terminal = InputTerminal {
            id: start_id + 1,
            terminal_type: self.terminal_type,
            assoc_terminal: start_id,
            clock_source: self.clock_source_id,
            num_channels: self.num_channels,
            channel_config: self.channel_config,
            channel_names: 0,
            copy_protect_control: AccessControl::NotPresent,
            connector_control: AccessControl::NotPresent,
            cluster_control: AccessControl::NotPresent,
            overload_control: AccessControl::NotPresent,
            underflow_control: AccessControl::NotPresent,
            overflow_control: AccessControl::NotPresent,
            phantom_power_control: AccessControl::NotPresent,
            string: self.string,
        };
        (output_terminal, input_terminal)
    }
    fn write_interface_descriptors(
        &self,
        writer: &mut DescriptorWriter,
        if_id: InterfaceNumber,
    ) -> usb_device::Result<()> {
        writer.interface(
            if_id,
            AUDIO,
            InterfaceSubclass::AudioStreaming as u8,
            InterfaceProtocol::Version2 as u8,
        )?;
        writer.interface_alt(
            if_id,
            1,
            AUDIO,
            InterfaceSubclass::AudioStreaming as u8,
            InterfaceProtocol::Version2 as u8,
            None,
        )?;

        // TODO:
        // 1. Interface specific AS_GENERAL descriptor (4.9.2)
        // 2. Format Type I descriptor
        // 3. Endpoint descriptor

        Ok(())
    }
}

/// Configuration and references to the Audio Class descriptors
///
/// Supports one clock source, optionally one input terminal and optionally one output terminal.
/// An optional set of additional descriptors can be provided, but must be handled by the user.
///
/// The two Terminal descriptors will be built per their TerminalConfig
///
/// Unit IDs will be fixed as follows:
/// * Clock Source: 1
/// * USB Streaming Input: 2
/// * Output Terminal: 3
/// * USB Streaming Output: 4
/// * Input Terminal: 5
/// * User provided descriptors: 6+
///
/// A single Clock Source is always required, but a fully custom descriptor set can be built by only providing
/// the Clock Source and additional descriptors, if the Terminal descriptors are inappropriate.
///
pub struct AudioClassConfig<'a, CS: UsbAudioClockSource> {
    pub device_category: FunctionCode,
    pub clock: CS,
    pub input_config: Option<TerminalConfig<Out>>,
    pub output_config: Option<TerminalConfig<In>>,
    pub additional_descriptors: Option<&'a [AudioClassDescriptor]>,
}

impl<'a, CS: UsbAudioClockSource> AudioClassConfig<'a, CS> {
    pub fn new(device_category: FunctionCode, clock: CS) -> Self {
        Self {
            device_category,
            clock,
            input_config: None,
            output_config: None,
            additional_descriptors: None,
        }
    }
    pub fn with_input_config(mut self, input_config: TerminalConfig<Out>) -> Self {
        self.input_config = Some(input_config);
        self
    }
    pub fn with_output_terminal(mut self, output_terminal: TerminalConfig<In>) -> Self {
        self.output_config = Some(output_terminal);
        self
    }
    pub fn with_additional_descriptors(
        mut self,
        additional_descriptors: &'a [AudioClassDescriptor],
    ) -> Self {
        self.additional_descriptors = Some(additional_descriptors);
        self
    }
    /// Writes the class-specific configuration descriptor set (after bDescriptortype INTERFACE)
    fn get_configuration_descriptors(
        &self,
        writer: &mut DescriptorWriter<'_>,
    ) -> usb_device::Result<()> {
        // CONFIGURATION DESCRIPTORS //
        let mut total_length: u16 = 9; // HEADER
        let clock_desc = self.clock.get_configuration_descriptor(1, None)?;
        total_length += clock_desc.size() as u16;
        let output_descs = match &self.output_config {
            Some(config) => {
                let descs = config.get_configuration_descriptors(2);
                total_length += descs.0.size() as u16 + descs.1.size() as u16;
                Some(descs)
            }
            None => None,
        };
        let input_descs = match &self.input_config {
            Some(config) => {
                let descs = config.get_configuration_descriptors(4);
                total_length += descs.0.size() as u16 + descs.1.size() as u16;
                Some(descs)
            }
            None => None,
        };
        let additional_descs = match &self.additional_descriptors {
            Some(descs) => {
                total_length += descs.iter().map(|desc| desc.size() as u16).sum::<u16>();
                Some(descs)
            }
            None => None,
        };
        let ac_header: [u8; 7] = [
            ClassSpecificACInterfaceDescriptorSubtype::Header as u8,
            0,                                  // bcdADC[0]
            2,                                  // bcdADC[1]
            self.device_category as u8,         // bCategory
            (total_length & 0xff) as u8,        // wTotalLength LSB
            ((total_length >> 8) & 0xff) as u8, // wTotalLength MSB
            0,                                  // bmControls
        ];
        writer.write(ClassSpecificDescriptorType::Interface as u8, &ac_header)?;
        clock_desc.write_descriptor(writer)?;
        if let Some((a, b)) = output_descs {
            a.write_descriptor(writer)?;
            b.write_descriptor(writer)?;
        }
        if let Some((a, b)) = input_descs {
            a.write_descriptor(writer)?;
            b.write_descriptor(writer)?;
        }
        if let Some(descs) = additional_descs {
            for desc in descs.into_iter() {
                desc.write_descriptor(writer)?;
            }
        }

        // INTERFACE DESCRIPTORS //

        Ok(())
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
    interface: InterfaceNumber,
    endpoint: Endpoint<'a, B, D>,
    alt_setting: u8,
}

pub struct AudioClass<'a, CS: UsbAudioClockSource> {
    control_iface: InterfaceNumber,
    config: AudioClassConfig<'a, CS>,
}

impl<CS: UsbAudioClockSource> AudioClass<'_, CS> {}

impl<B: UsbBus, CS: UsbAudioClockSource> UsbClass<B> for AudioClass<'_, CS> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut DescriptorWriter,
    ) -> usb_device::Result<()> {
        // IN, OUT, CONTROL
        let n_interfaces = self.config.input_config.is_some() as u8
            + self.config.output_config.is_some() as u8
            + 1;

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

        self.config.get_configuration_descriptors(writer)?;

        Ok(())
    }
}
