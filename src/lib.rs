#![no_std]
#![allow(dead_code)]

pub mod constants;
mod cursor;
pub mod descriptors;
mod log;

use core::cmp::Ordering;
use core::marker::PhantomData;

use byteorder_embedded_io::{LittleEndian, WriteBytesExt};
use constants::*;
use descriptors::*;
use log::*;

use num_traits::{ConstZero, Zero};
use usb_device::control::{Recipient, Request, RequestType};
use usb_device::device::DEFAULT_ALTERNATE_SETTING;
use usb_device::endpoint::{self, Endpoint, EndpointDirection, In, Out};
use usb_device::{UsbDirection, class_prelude::*};

pub use constants::USB_CLASS_AUDIO;

#[cfg(feature = "defmt")]
use defmt;

mod sealed {
    pub trait Sealed {}
}

#[derive(Debug)]
pub enum UsbAudioClassError {
    NotImplemented,
    Other,
}

impl<T: core::error::Error> From<T> for UsbAudioClassError {
    fn from(_: T) -> Self {
        UsbAudioClassError::Other
    }
}

impl From<UsbAudioClassError> for UsbError {
    fn from(_value: UsbAudioClassError) -> Self {
        UsbError::Unsupported
    }
}

pub trait RangeType:
    sealed::Sealed + num_traits::PrimInt + num_traits::ToBytes + ConstZero
{
}
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

#[derive(PartialEq, Eq, Ord)]
pub struct RangeEntry<T: RangeType> {
    pub min: T,
    pub max: T,
    pub res: T,
}

impl<T: RangeType> RangeEntry<T> {
    pub const fn new(min: T, max: T, res: T) -> Self {
        Self { min, max, res }
    }
    pub const fn new_fixed(fixed: T) -> Self {
        Self {
            min: fixed,
            max: fixed,
            res: T::ZERO,
        }
    }

    pub fn write<W: embedded_io::Write>(
        &self,
        mut buf: W,
    ) -> core::result::Result<usize, W::Error> {
        buf.write_all(self.min.to_le_bytes().as_ref())?;
        buf.write_all(self.max.to_le_bytes().as_ref())?;
        buf.write_all(self.res.to_le_bytes().as_ref())?;
        Ok(T::ZERO.count_zeros() as usize * 3)
    }
}

/// The spec guarantees that ranges do not overlap, so compare by min is correct.
impl<T: RangeType + PartialOrd> PartialOrd for RangeEntry<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.min.partial_cmp(&other.min)
    }
}

/// A trait for implementing USB Audio Class 2 devices
///
/// Contains callback methods which will be called by the class driver. All
/// callbacks are optional, which may be useful for a tight-loop polling implementation
/// but most implementations will want to implement at least `audio_data_rx`.
///
/// Unimplemented callbacks should return `Ok(())` if a result is required.

pub trait UsbAudioClass<'a, B: UsbBus> {
    /// Called when audio data is received from the host. `ep` is ready for
    /// `ep.read()`.
    fn audio_data_rx(&self, ep: &Endpoint<'a, B, endpoint::Out>) {}

    /// Called when the alternate setting of `terminal`'s interface is changed,
    /// before the `AudioStream` is updated. Currently not very useful since we
    /// don't implement alternate settings.
    fn alternate_setting_changed<CS: UsbAudioClockImpl, AU: UsbAudioClass<'a, B>>(
        &self,
        ac: &mut AudioClass<'a, B, CS, AU>,
        terminal: UsbDirection,
        alt_setting: u8,
    ) {
    }
}

/// A trait for implementing Sampling Frequency Control for USB Audio Clock Sources
/// ref: USB Audio Class Specification 2.0 5.2.5.1.1
///
/// Contains callback methods which will be called by the class driver.
///
/// Unimplemented callbacks should return `Err(UsbAudioClassError::NotImplemented)`. Other
/// errors will panic (the underlying callbacks are not fallible). If you need to handle errors,
/// you should use the callback to infalliably signal another task.
pub trait UsbAudioClockImpl {
    const CLOCK_TYPE: ClockType;
    const SOF_SYNC: bool;
    /// Called when the host requests the current sample rate. Returns the sample rate in Hz.
    fn get_sample_rate(&self) -> core::result::Result<u32, UsbAudioClassError>;
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

    /// Called during allocation and when the hosts makes a RANGE request for the clock source.
    ///
    /// Returns a slice of possible sample rates.
    ///
    /// Rates must meet the invariants in the specification:
    ///   * The subranges must be ordered in ascendingorder
    ///   * Individual subranges cannot overlap
    ///   * If a subrange consists of only a single value, the corresponding triplet must contain that value for both
    ///     its MIN and MAX subattribute and the RES subattribute must be set to zero
    ///
    /// ref: USB Audio Class Specification 2.0 5.2.1 & 5.2.3.3
    fn get_rates(&self) -> core::result::Result<&[RangeEntry<u32>], UsbAudioClassError>;

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

// This trait is needed since we specialize on D
trait TerminalConfigurationDescriptors {
    fn get_configuration_descriptors(&self) -> (InputTerminal, OutputTerminal);
}

pub struct TerminalConfig<D: EndpointDirection> {
    /// USB terminal in the D direction will have this id, audio terminal will have this id + 1
    base_id: u8,
    clock_source_id: u8,
    num_channels: u8,
    format: FormatType1,
    terminal_type: TerminalType,
    channel_config: ChannelConfig,
    sync_type: IsochronousSynchronizationType,
    lock_delay: LockDelay,
    string: Option<StringIndex>,
    _direction: PhantomData<D>,
}

// TODO: builder pattern
impl<D: EndpointDirection> TerminalConfig<D> {
    pub fn new(
        base_id: u8,
        clock_source_id: u8,
        num_channels: u8,
        format: FormatType1,
        terminal_type: TerminalType,
        channel_config: ChannelConfig,
        sync_type: IsochronousSynchronizationType,
        lock_delay: LockDelay,
        string: Option<StringIndex>,
    ) -> Self {
        TerminalConfig {
            base_id,
            clock_source_id,
            num_channels,
            format,
            terminal_type,
            channel_config,
            sync_type,
            lock_delay,
            string,
            _direction: PhantomData,
        }
    }
    // Bytes per audio frame
    pub fn bytes_per_frame(&self) -> u32 {
        self.format.bytes_per_sample as u32 * self.num_channels as u32
    }
}
impl<'a> TerminalConfigurationDescriptors for TerminalConfig<In> {
    fn get_configuration_descriptors(&self) -> (InputTerminal, OutputTerminal) {
        let input_terminal = InputTerminal {
            id: self.base_id,
            terminal_type: TerminalType::UsbStreaming,
            assoc_terminal: 0,
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
            string: self.string,
        };
        let output_terminal = OutputTerminal {
            id: self.base_id + 1,
            terminal_type: self.terminal_type,
            assoc_terminal: 0,
            source_id: self.base_id,
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

impl<'a> TerminalConfigurationDescriptors for TerminalConfig<Out> {
    fn get_configuration_descriptors(&self) -> (InputTerminal, OutputTerminal) {
        let output_terminal = OutputTerminal {
            id: self.base_id,
            terminal_type: TerminalType::UsbStreaming,
            assoc_terminal: 0,
            source_id: self.base_id + 1,
            clock_source: self.clock_source_id,
            copy_protect_control: AccessControl::NotPresent,
            connector_control: AccessControl::NotPresent,
            overload_control: AccessControl::NotPresent,
            underflow_control: AccessControl::NotPresent,
            overflow_control: AccessControl::NotPresent,
            string: self.string,
        };
        let input_terminal = InputTerminal {
            id: self.base_id + 1,
            terminal_type: self.terminal_type,
            assoc_terminal: 0,
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
        (input_terminal, output_terminal)
    }
}

#[derive(Copy, Clone, Debug)]
pub enum UsbSpeed {
    Low, // Not supported for audio
    Full,
    High,
    Super,
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
pub struct AudioClassConfig<'a, B: UsbBus, CS: UsbAudioClockImpl, AU: UsbAudioClass<'a, B>> {
    pub speed: UsbSpeed,
    pub device_category: FunctionCode,
    pub clock_impl: &'a CS,
    pub audio_impl: &'a AU,
    pub input_config: Option<TerminalConfig<Out>>,
    pub output_config: Option<TerminalConfig<In>>,
    pub additional_descriptors: Option<&'a [AudioClassDescriptor]>,
    _bus: PhantomData<B>,
}

impl<'a, B: UsbBus, CS: UsbAudioClockImpl, AU: UsbAudioClass<'a, B>>
    AudioClassConfig<'a, B, CS, AU>
{
    pub fn new(
        speed: UsbSpeed,
        device_category: FunctionCode,
        clock_impl: &'a CS,
        audio_impl: &'a AU,
    ) -> Self {
        Self {
            speed,
            device_category,
            clock_impl,
            audio_impl,
            input_config: None,
            output_config: None,
            additional_descriptors: None,
            _bus: PhantomData,
        }
    }
    pub fn with_input_config(mut self, input_config: TerminalConfig<Out>) -> Self {
        self.input_config = Some(input_config);
        self
    }
    pub fn with_output_config(mut self, output_config: TerminalConfig<In>) -> Self {
        self.output_config = Some(output_config);
        self
    }
    pub fn with_additional_descriptors(
        mut self,
        additional_descriptors: &'a [AudioClassDescriptor],
    ) -> Self {
        self.additional_descriptors = Some(additional_descriptors);
        self
    }

    /// Allocate the various USB IDs, and build the class implementation
    pub fn build(self, alloc: &'a UsbBusAllocator<B>) -> Result<AudioClass<'a, B, CS, AU>> {
        let speed = self.speed;
        let interval = match speed {
            UsbSpeed::Full => 1,
            UsbSpeed::High | UsbSpeed::Super => 4, // rate = 2^(4-1) * 125us = 1ms, same as full speed
            UsbSpeed::Low => return Err(Error::InvalidSpeed),
        };
        let max_rate = self
            .clock_impl
            .get_rates()
            .unwrap()
            .iter()
            .max()
            .unwrap()
            .max;
        let control_iface = alloc.interface();

        let mut ac = AudioClass {
            control_iface,
            clock_impl: self.clock_impl,
            audio_impl: self.audio_impl,
            output: None,
            input: None,
            feedback: None,
            additional_descriptors: self.additional_descriptors,
            device_category: self.device_category,
            in_iface: 0,
            out_iface: 0,
            in_ep: 0,
            out_ep: 0,
            fb_ep: 0,
        };

        if let Some(config) = self.output_config {
            let interface = alloc.interface();
            let endpoint = alloc.isochronous(
                config.sync_type,
                IsochronousUsageType::Data,
                ((config.bytes_per_frame() * max_rate) / 1000) as u16,
                interval,
            );
            let feedback_ep = alloc.isochronous(
                IsochronousSynchronizationType::NoSynchronization,
                IsochronousUsageType::Feedback,
                4,
                interval,
            );
            let alt_setting = DEFAULT_ALTERNATE_SETTING;
            ac.in_iface = interface.into();
            ac.in_ep = endpoint.address().index();
            ac.fb_ep = feedback_ep.address().index();
            ac.input = Some(AudioStream {
                config,
                interface,
                endpoint,
                alt_setting,
            });
            ac.feedback = Some(feedback_ep);
        }

        if let Some(config) = self.input_config {
            let interface = alloc.interface();
            let endpoint = alloc.isochronous(
                config.sync_type,
                IsochronousUsageType::Data,
                ((config.bytes_per_frame() * max_rate) / 1000) as u16,
                interval,
            );
            let alt_setting = DEFAULT_ALTERNATE_SETTING;
            ac.out_iface = interface.into();
            ac.out_ep = endpoint.address().index();
            ac.output = Some(AudioStream {
                config,
                interface,
                endpoint,
                alt_setting,
            });
        }

        Ok(ac)
    }
}

/// USB audio errors, including possible USB Stack errors
#[derive(Debug)]
pub enum Error {
    InvalidValue,
    BandwidthExceeded,
    StreamNotInitialized,
    InvalidSpeed,
    UsbError(usb_device::UsbError),
}

impl From<UsbError> for Error {
    fn from(err: UsbError) -> Self {
        Error::UsbError(err)
    }
}
type Result<T> = core::result::Result<T, Error>;

struct AudioStream<'a, B: UsbBus, D: EndpointDirection> {
    // UsbStreaming terminal ID
    config: TerminalConfig<D>,
    interface: InterfaceNumber,
    endpoint: Endpoint<'a, B, D>,
    alt_setting: u8,
}

impl<'a, B: UsbBus, D: EndpointDirection> AudioStream<'a, B, D> {
    fn write_interface_descriptors(&self, writer: &mut DescriptorWriter) -> usb_device::Result<()> {
        debug!(
            "  AudioStream<{}>.write_interface_descriptors",
            core::any::type_name::<D>()
        );
        // UAC2 4.9.1 Standard AS Interface Descriptor
        //   zero bandwidth configuration per 3.16.2
        //
        // Whenever an AudioStreaming interface requires an isochronous data
        // endpoint, it must at least provide the default Alternate Setting
        // (Alternate Setting 0) with zero bandwidth requirements (no
        // isochronous data endpoint defined) and an additional Alternate
        // Setting that contains the actual isochronous data endpoint.
        debug!("writer.interface AudioStreaming");
        writer.interface(
            self.interface,
            USB_CLASS_AUDIO,
            InterfaceSubclass::AudioStreaming as u8,
            InterfaceProtocol::Version2 as u8,
        )?;
        // UAC2 4.9.1 Standard AS Interface Descriptor
        //   live data configuration
        debug!("writer.interface_alt AudioStreaming");
        writer.interface_alt(
            self.interface,
            1,
            USB_CLASS_AUDIO,
            InterfaceSubclass::AudioStreaming as u8,
            InterfaceProtocol::Version2 as u8,
            None,
        )?;

        // UAC2 4.9.2 Class-Specific AS Interface Descriptor
        let as_general = AudioStreamingInterface {
            terminal_id: self.config.base_id, // Always the USB streaming terminal id
            active_alt_setting: AccessControl::NotPresent,
            valid_alt_settings: AccessControl::NotPresent,
            format_type: FormatType::Type1,
            format_bitmap: Type1FormatBitmap::Pcm, // only PCM is supported
            num_channels: self.config.num_channels,
            channel_config: self.config.channel_config,
            string: self.config.string,
        };
        as_general.write_descriptor(writer)?;

        // UAC2 4.9.3 Class-Specific AS Format Type Descriptor
        self.config.format.write_descriptor(writer)?;

        Ok(())
    }

    fn write_endpoint_descriptors(&self, writer: &mut DescriptorWriter) -> usb_device::Result<()> {
        // UAC2 4.10.1.1 Standard AS Isochronous Audio Data Endpoint Descriptor
        debug!(
            "  AudioStream<{}>.write_endpoint_descriptors",
            core::any::type_name::<D>()
        );
        debug!("writer.endpoint");
        writer.endpoint(&self.endpoint)?;
        // UAC2 4.10.1.2 Class-Specific AS Isochronous Audio Data Endpoint Descriptor
        let cs_ep = AudioStreamingEndpoint {
            max_packets_only: false,
            pitch_control: AccessControl::NotPresent,
            overrun_control: AccessControl::NotPresent,
            underrun_control: AccessControl::NotPresent,
            lock_delay: self.config.lock_delay,
        };
        cs_ep.write_descriptor(writer)?;
        Ok(())
    }
}

pub struct AudioClass<'a, B: UsbBus, CS: UsbAudioClockImpl, AU: UsbAudioClass<'a, B>> {
    control_iface: InterfaceNumber,
    clock_impl: &'a CS,
    audio_impl: &'a AU,
    output: Option<AudioStream<'a, B, Out>>,
    input: Option<AudioStream<'a, B, In>>,
    feedback: Option<Endpoint<'a, B, In>>,
    additional_descriptors: Option<&'a [AudioClassDescriptor]>,
    device_category: FunctionCode,
    in_iface: u8,
    out_iface: u8,
    in_ep: usize,
    out_ep: usize,
    fb_ep: usize,
}

impl<'a, B: UsbBus, CS: UsbAudioClockImpl, AU: UsbAudioClass<'a, B>> UsbClass<B>
    for AudioClass<'a, B, CS, AU>
{
    fn get_configuration_descriptors(
        &self,
        writer: &mut DescriptorWriter<'_>,
    ) -> usb_device::Result<()> {
        info!("  AudioClass::get_configuration_descriptors");
        // Control + 0-2 streaming
        let n_interfaces = 1 + (self.input.is_some() as u8) + (self.input.is_some() as u8);

        debug!("writer.iad()");
        // UAC2 4.6 Interface Association Descriptor
        writer.iad(
            self.control_iface,
            n_interfaces,
            USB_CLASS_AUDIO,
            InterfaceSubclass::Undefined as u8,
            FunctionProtocol::Version2 as u8,
            None,
        )?;

        debug!("writer.interface()");
        // UAC2 4.7 Standard AC Interface Descriptor
        writer.interface(
            self.control_iface,
            USB_CLASS_AUDIO,
            InterfaceSubclass::AudioControl as u8,
            InterfaceProtocol::Version2 as u8,
        )?;

        // BUILD CONFIGURATION DESCRIPTORS //
        let mut total_length: u16 = 9; // HEADER
        let clock_desc = self.clock_impl.get_configuration_descriptor(1, None)?;
        total_length += clock_desc.size() as u16;
        let output_descs = match &self.output {
            Some(stream) => {
                let descs = stream.config.get_configuration_descriptors();
                total_length += descs.0.size() as u16 + descs.1.size() as u16;
                Some(descs)
            }
            None => None,
        };
        let input_descs = match &self.input {
            Some(stream) => {
                let descs = stream.config.get_configuration_descriptors();
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
        debug!(
            "have output: {}, have input: {}, have additional: {}, total length: {}",
            output_descs.is_some(),
            input_descs.is_some(),
            additional_descs.is_some(),
            total_length
        );

        // UAC2 4.7.2 Class-specific AC Interface Descriptor
        let ac_header: [u8; 7] = [
            ClassSpecificACInterfaceDescriptorSubtype::Header as u8,
            0,                                  // bcdADC[0]
            2,                                  // bcdADC[1]
            self.device_category as u8,         // bCategory
            (total_length & 0xff) as u8,        // wTotalLength LSB
            ((total_length >> 8) & 0xff) as u8, // wTotalLength MSB
            0,                                  // bmControls
        ];
        debug!("writer.write (AC header)");
        writer.write(ClassSpecificDescriptorType::Interface as u8, &ac_header)?;

        // UAC2 4.7.2.1 Clock Source Descriptor
        clock_desc.write_descriptor(writer)?;

        // UAC2 4.7.2.4 & 4.7.2.5 Input & Output Terminal Descriptors
        if let Some((a, b)) = output_descs {
            a.write_descriptor(writer)?;
            b.write_descriptor(writer)?;
        }
        if let Some((a, b)) = input_descs {
            a.write_descriptor(writer)?;
            b.write_descriptor(writer)?;
        }

        // UAC2 4.7
        if let Some(descs) = additional_descs {
            for desc in descs.into_iter() {
                desc.write_descriptor(writer)?;
            }
        }

        // UAC2 4.9 & 2.4.10
        if let Some(stream) = &self.input {
            stream.write_interface_descriptors(writer)?;
            stream.write_endpoint_descriptors(writer)?;
        }
        if let Some(stream) = &self.output {
            stream.write_interface_descriptors(writer)?;
            stream.write_endpoint_descriptors(writer)?;
        }
        // UAC2 4.9.2.1 Feedback Endpoint Descriptor
        //   Should always be present if an OUT endpoint is present
        if let Some(feedback) = &self.feedback {
            debug!("writer.endpoint (feedback)");
            writer.endpoint(feedback)?;
        }

        Ok(())
    }
    fn control_out(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();
        match req.request_type {
            RequestType::Standard => self.standard_request_out(xfer),
            RequestType::Class => self.class_request_out(xfer),
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }
    fn control_in(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();
        match req.request_type {
            RequestType::Standard => self.standard_request_in(xfer),
            RequestType::Class => self.class_request_in(xfer),
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }
    fn endpoint_out(&mut self, addr: EndpointAddress) {
        debug!("EP {} out data", addr);
        if addr.index() == self.out_ep {
            self.audio_impl
                .audio_data_rx(&self.output.as_ref().unwrap().endpoint)
        } else {
            debug!("  unexpected OUT on {}", addr);
        }
    }

    fn poll(&mut self) {
        debug!("poll");
        // no streaming in alt 0
        if self.output.as_ref().unwrap().alt_setting != 1 {
            return;
        }
        loop {
            let mut buf = [0; 1024];
            match self.output.as_ref().unwrap().endpoint.read(&mut buf) {
                Ok(len) if len > 0 => {
                    debug!("EP OUT data {:?}", len);
                }
                Ok(_) => {
                    debug!("EP OUT empty");
                    break;
                }
                Err(UsbError::WouldBlock) => break,
                Err(err) => {
                    debug!("EP OUT error {:?}", err);
                }
            }
        }
    }
}

impl<'a, B: UsbBus, CS: UsbAudioClockImpl, AU: UsbAudioClass<'a, B>> AudioClass<'a, B, CS, AU> {
    fn standard_request_out(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();
        match (req.recipient, req.request) {
            (Recipient::Interface, Request::SET_INTERFACE) => self.set_alt_interface(xfer),
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }

    fn standard_request_in(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();
        match (req.recipient, req.request) {
            (Recipient::Interface, Request::GET_INTERFACE) => self.get_alt_interface(xfer),
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }

    fn class_request_out(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();
        match (req.recipient, req.request.try_into()) {
            (Recipient::Interface, Ok(ClassSpecificRequest::Cur)) => self.set_interface_cur(xfer),
            (Recipient::Interface, Ok(ClassSpecificRequest::Range)) => {}
            (Recipient::Endpoint, Ok(ClassSpecificRequest::Cur)) => self.set_endpoint_cur(xfer),
            (Recipient::Endpoint, Ok(ClassSpecificRequest::Range)) => {}
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }

    fn class_request_in(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();

        match (req.recipient, req.request.try_into()) {
            (Recipient::Interface, Ok(ClassSpecificRequest::Cur)) => self.get_interface_cur(xfer),
            (Recipient::Interface, Ok(ClassSpecificRequest::Range)) => {
                self.get_interface_range(xfer)
            }
            (Recipient::Endpoint, Ok(ClassSpecificRequest::Cur)) => self.get_endpoint_cur(xfer),
            (Recipient::Endpoint, Ok(ClassSpecificRequest::Range)) => self.get_endpoint_range(xfer),
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }

    fn set_alt_interface(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();
        let iface = req.index as u8;
        let alt_setting = req.value as u8;

        debug!("  SET_ALT_INTERFACE {} {}", iface, alt_setting);

        if self.input.is_some() && iface == self.in_iface {
            let old_alt = self.input.as_ref().unwrap().alt_setting;
            if old_alt != alt_setting {
                self.audio_impl
                    .alternate_setting_changed(self, UsbDirection::In, alt_setting);
                self.input.as_mut().unwrap().alt_setting = alt_setting;
                xfer.accept().ok();
            }
        } else if self.output.is_some() && iface == self.out_iface {
            let old_alt = self.output.as_ref().unwrap().alt_setting;
            if old_alt != alt_setting {
                self.audio_impl
                    .alternate_setting_changed(self, UsbDirection::Out, alt_setting);
                self.output.as_mut().unwrap().alt_setting = alt_setting;
                xfer.accept().ok();
            }
        } else {
            debug!(
                "   not handled (in: {}, out: {}, got: {}).",
                self.in_iface, self.out_iface, iface
            );
        }
    }
    fn get_alt_interface(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();
        let iface = req.index as u8;
        debug!("  GET_ALT_INTERFACE {}", iface);
        if self.input.is_some() && iface == self.in_iface {
            xfer.accept_with(&[self.input.as_ref().unwrap().alt_setting])
                .ok();
            return;
        } else if self.output.is_some() && iface == self.out_iface {
            xfer.accept_with(&[self.output.as_ref().unwrap().alt_setting])
                .ok();
            return;
        }
        debug!("   Unimplemented.");
    }
    fn get_interface_cur(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();
        let entity = (req.index >> 8) as u8;
        let interface = (req.index & 0xff) as u8;
        let control = (req.value >> 8) as u8;
        let channel = (req.value & 0xff) as u8;

        debug!(
            "  GET_INTERFACE_CUR entity: {} iface: {} control: {} channel: {}",
            entity, interface, control, channel
        );
        if interface == self.control_iface.into() {
            return self.get_control_interface_cur(xfer, entity, channel, control);
        }
        debug!("   Unimpleneted.");
    }
    fn set_interface_cur(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();
        let entity = (req.index >> 8) as u8;
        let interface = (req.index & 0xff) as u8;
        let control = (req.value >> 8) as u8;
        let channel = (req.value & 0xff) as u8;

        debug!(
            "  SET_INTERFACE_CUR entity: {} iface: {} control: {} channel: {}",
            entity, interface, control, channel
        );

        if interface == self.control_iface.into() {
            return self.set_control_interface_cur(xfer, entity, channel, control);
        } else if interface == self.in_iface {
            return self.set_streaming_interface_cur(
                xfer,
                UsbDirection::In,
                entity,
                channel,
                control,
            );
        } else if interface == self.out_iface {
            return self.set_streaming_interface_cur(
                xfer,
                UsbDirection::Out,
                entity,
                channel,
                control,
            );
        }
        debug!("   Unimplemented.");
    }

    fn get_control_interface_cur(
        &mut self,
        xfer: ControlIn<B>,
        entity: u8,
        channel: u8,
        control: u8,
    ) {
        match entity {
            1 => return self.get_clock_cur(xfer, channel, control),
            _ => {}
        }
        debug!("   Unimplemented.");
    }

    fn set_control_interface_cur(
        &mut self,
        xfer: ControlOut<B>,
        entity: u8,
        channel: u8,
        control: u8,
    ) {
        debug!("   Unimplemented.");
    }

    fn set_streaming_interface_cur(
        &mut self,
        xfer: ControlOut<B>,
        direction: UsbDirection,
        entity: u8,
        channel: u8,
        control: u8,
    ) {
        debug!("   Unimplemented.");
    }

    fn get_endpoint_cur(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();
        let entity = (req.index >> 8) as u8;
        let interface = (req.index & 0xff) as u8;
        let control = (req.value >> 8) as u8;
        let channel = (req.value & 0xff) as u8;

        debug!("   Unimplemented.");
    }
    fn get_endpoint_range(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();
        let entity = (req.index >> 8) as u8;
        let interface = (req.index & 0xff) as u8;
        let control = (req.value >> 8) as u8;
        let channel = (req.value & 0xff) as u8;

        debug!("   Unimplemented.");
    }

    fn set_endpoint_cur(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();
        let entity = (req.index >> 8) as u8;
        let interface = (req.index & 0xff) as u8;
        let control = (req.value >> 8) as u8;
        let channel = (req.value & 0xff) as u8;

        debug!("   Unimplemented.");
    }

    fn get_interface_range(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();

        let entity = (req.index >> 8) as u8;
        let interface = (req.index & 0xff) as u8;
        let control = (req.value >> 8) as u8;
        let channel = (req.value & 0xff) as u8;

        debug!(
            "  GET_INTERFACE_RANGE entity: {} iface: {} control: {} channel: {}",
            entity, interface, control, channel
        );
        if interface == self.control_iface.into() {
            return self.get_control_interface_range(xfer, entity, channel, control);
        }
        debug!("   Unimplemented.");
    }

    fn get_control_interface_range(
        &mut self,
        xfer: ControlIn<B>,
        entity: u8,
        channel: u8,
        control: u8,
    ) {
        match entity {
            1 => self.get_clock_range(xfer, channel, control), // clock source
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }
    fn get_clock_cur(&mut self, xfer: ControlIn<B>, channel: u8, control: u8) {
        match control.try_into() {
            Ok(ClockSourceControlSelector::SamFreqControl) => {
                debug!("   SamplingFreqControl");
                if channel != 0 {
                    error!(
                        "   Invalid channel {} for SamplingFreqControl GET CUR. Ignoring.",
                        channel
                    );
                }
                xfer.accept(|mut buf| match self.clock_impl.get_sample_rate() {
                    Ok(rate) => {
                        debug!("    {}", rate);
                        buf.write_u32::<LittleEndian>(rate)
                            .map_err(|_e| UsbError::BufferOverflow)?;
                        Ok(4)
                    }
                    Err(_e) => Err(UsbError::InvalidState),
                })
                .ok();
            }
            Ok(ClockSourceControlSelector::ClockValidControl) => {
                debug!("   ClockValidControl");
                if channel != 0 {
                    error!(
                        "   Invalid channel {} for ClockValidControl GET CUR. Ignoring.",
                        channel
                    );
                }
                xfer.accept(|mut buf| match self.clock_impl.get_clock_validity() {
                    Ok(valid) => {
                        debug!("    {}", valid);
                        buf.write_u8(valid as u8)
                            .map_err(|_e| UsbError::BufferOverflow)?;
                        Ok(1)
                    }
                    Err(_e) => Err(UsbError::InvalidState),
                })
                .ok();
            }
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }
    fn get_clock_range(&mut self, xfer: ControlIn<B>, channel: u8, control: u8) {
        match control.try_into() {
            Ok(ClockSourceControlSelector::SamFreqControl) => {
                debug!("   SamplingFreqControl");
                if channel != 0 {
                    error!(
                        "   Invalid channel {} for SamplingFreqControl GET RANGE. Ignoring.",
                        channel
                    );
                }
                xfer.accept(|mut buf| match self.clock_impl.get_rates() {
                    Ok(rates) => {
                        buf.write_u16::<LittleEndian>(rates.len() as u16)
                            .map_err(|_e| UsbError::BufferOverflow)?;
                        let mut written = 2usize;
                        for rate in rates {
                            written += rate
                                .write(&mut buf)
                                .map_err(|_e| UsbError::BufferOverflow)?
                        }
                        Ok(written)
                    }
                    Err(_) => Err(UsbError::InvalidState),
                })
                .ok();
            }
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }
    pub fn read(&mut self, buf: &mut [u8]) -> usb_device::Result<usize> {
        match self.output.as_mut().unwrap().endpoint.read(buf) {
            Ok(len) => {
                debug!("NO CB read {} bytes", len);
                Ok(len)
            }
            Err(UsbError::WouldBlock) => Err(UsbError::WouldBlock),
            Err(e) => {
                error!("read error");
                Err(e)
            }
        }
    }
}
