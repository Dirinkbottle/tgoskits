#![no_std]

#[macro_use]
extern crate alloc;

use alloc::{string::String, vec::Vec};

use anyhow::anyhow;
use crab_usb::{Device, DeviceInfo, err::USBError};
use log::*;
use usb_if::{
    descriptor::{Class, EndpointType},
    host::ControlSetup,
    transfer::{Direction, Recipient, Request, RequestType},
};

// 导入描述符解析模块
pub mod descriptors;
pub use descriptors::*;

pub mod stream;
// 帧解析模块（参考 libuvc 的包头解析与帧组装）
pub mod frame;

pub use crate::stream::VideoStream;

// 保持向后兼容的常量别名
pub mod uvc_requests {
    pub use crate::descriptors::request_codes::*;
}

pub mod pu_controls {
    pub use crate::descriptors::processing_unit_controls::*;
    // 添加原有的常量别名
    pub const PU_BRIGHTNESS_CONTROL: u8 = super::descriptors::processing_unit_controls::BRIGHTNESS;
    pub const PU_CONTRAST_CONTROL: u8 = super::descriptors::processing_unit_controls::CONTRAST;
    pub const PU_HUE_CONTROL: u8 = super::descriptors::processing_unit_controls::HUE;
    pub const PU_SATURATION_CONTROL: u8 = super::descriptors::processing_unit_controls::SATURATION;
    pub const PU_SHARPNESS_CONTROL: u8 = super::descriptors::processing_unit_controls::SHARPNESS;
    pub const PU_GAMMA_CONTROL: u8 = super::descriptors::processing_unit_controls::GAMMA;
    pub const PU_WHITE_BALANCE_TEMPERATURE_CONTROL: u8 =
        super::descriptors::processing_unit_controls::WHITE_BALANCE_TEMPERATURE;
    pub const PU_WHITE_BALANCE_COMPONENT_CONTROL: u8 =
        super::descriptors::processing_unit_controls::WHITE_BALANCE_COMPONENT;
    pub const PU_BACKLIGHT_COMPENSATION_CONTROL: u8 =
        super::descriptors::processing_unit_controls::BACKLIGHT_COMPENSATION;
    pub const PU_GAIN_CONTROL: u8 = super::descriptors::processing_unit_controls::GAIN;
    pub const PU_POWER_LINE_FREQUENCY_CONTROL: u8 =
        super::descriptors::processing_unit_controls::POWER_LINE_FREQUENCY;
    pub const PU_HUE_AUTO_CONTROL: u8 = super::descriptors::processing_unit_controls::HUE_AUTO;
    pub const PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL: u8 =
        super::descriptors::processing_unit_controls::WHITE_BALANCE_TEMPERATURE_AUTO;
    pub const PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL: u8 =
        super::descriptors::processing_unit_controls::WHITE_BALANCE_COMPONENT_AUTO;
}

pub mod vs_controls {
    pub use crate::descriptors::video_streaming_controls::*;
    // 添加原有的常量别名
    pub const VS_PROBE_CONTROL: u8 = super::descriptors::video_streaming_controls::PROBE;
    pub const VS_COMMIT_CONTROL: u8 = super::descriptors::video_streaming_controls::COMMIT;
    pub const VS_STILL_PROBE_CONTROL: u8 =
        super::descriptors::video_streaming_controls::STILL_PROBE;
    pub const VS_STILL_COMMIT_CONTROL: u8 =
        super::descriptors::video_streaming_controls::STILL_COMMIT;
}

pub mod terminal_types {
    pub use crate::descriptors::terminal_types::*;
}

pub mod uvc_descriptor_types {
    pub use crate::descriptors::descriptor_types::*;
}

pub mod uvc_interface_subtypes {
    // 保持原有命名
    pub const VC_DESCRIPTOR_UNDEFINED: u8 = super::descriptors::vc_descriptor_subtypes::UNDEFINED;
    pub const VC_HEADER: u8 = super::descriptors::vc_descriptor_subtypes::HEADER;
    pub const VC_INPUT_TERMINAL: u8 = super::descriptors::vc_descriptor_subtypes::INPUT_TERMINAL;
    pub const VC_OUTPUT_TERMINAL: u8 = super::descriptors::vc_descriptor_subtypes::OUTPUT_TERMINAL;
    pub const VC_SELECTOR_UNIT: u8 = super::descriptors::vc_descriptor_subtypes::SELECTOR_UNIT;
    pub const VC_PROCESSING_UNIT: u8 = super::descriptors::vc_descriptor_subtypes::PROCESSING_UNIT;
    pub const VC_EXTENSION_UNIT: u8 = super::descriptors::vc_descriptor_subtypes::EXTENSION_UNIT;

    pub const VS_UNDEFINED: u8 = super::descriptors::vs_descriptor_subtypes::UNDEFINED;
    pub const VS_INPUT_HEADER: u8 = super::descriptors::vs_descriptor_subtypes::INPUT_HEADER;
    pub const VS_OUTPUT_HEADER: u8 = super::descriptors::vs_descriptor_subtypes::OUTPUT_HEADER;
    pub const VS_STILL_IMAGE_FRAME: u8 =
        super::descriptors::vs_descriptor_subtypes::STILL_IMAGE_FRAME;
    pub const VS_FORMAT_UNCOMPRESSED: u8 =
        super::descriptors::vs_descriptor_subtypes::FORMAT_UNCOMPRESSED;
    pub const VS_FRAME_UNCOMPRESSED: u8 =
        super::descriptors::vs_descriptor_subtypes::FRAME_UNCOMPRESSED;
    pub const VS_FORMAT_MJPEG: u8 = super::descriptors::vs_descriptor_subtypes::FORMAT_MJPEG;
    pub const VS_FRAME_MJPEG: u8 = super::descriptors::vs_descriptor_subtypes::FRAME_MJPEG;
    pub const VS_FORMAT_MPEG2TS: u8 = super::descriptors::vs_descriptor_subtypes::FORMAT_MPEG2TS;
    pub const VS_FORMAT_DV: u8 = super::descriptors::vs_descriptor_subtypes::FORMAT_DV;
    pub const VS_COLORFORMAT: u8 = super::descriptors::vs_descriptor_subtypes::COLORFORMAT;
    pub const VS_FORMAT_FRAME_BASED: u8 =
        super::descriptors::vs_descriptor_subtypes::FORMAT_FRAME_BASED;
    pub const VS_FRAME_FRAME_BASED: u8 =
        super::descriptors::vs_descriptor_subtypes::FRAME_FRAME_BASED;
    pub const VS_FORMAT_STREAM_BASED: u8 =
        super::descriptors::vs_descriptor_subtypes::FORMAT_STREAM_BASED;
    pub const VS_FORMAT_H264: u8 = super::descriptors::vs_descriptor_subtypes::FORMAT_H264;
    pub const VS_FRAME_H264: u8 = super::descriptors::vs_descriptor_subtypes::FRAME_H264;
    pub const VS_FORMAT_H264_SIMULCAST: u8 =
        super::descriptors::vs_descriptor_subtypes::FORMAT_H264_SIMULCAST;
}

pub mod uvc_guids {
    pub use crate::descriptors::format_guids::*;
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VideoFormat {
    pub width: u16,
    pub height: u16,
    pub frame_rate: u32, // 帧率 (fps)
    pub format_type: VideoFormatType,
    /// Upper bound for one encoded or uncompressed frame, from the UVC frame
    /// descriptor or the negotiated VS Probe response.
    pub max_frame_size: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VideoFormatType {
    Uncompressed(UncompressedFormat),
    Mjpeg,
    H264,
}

impl VideoFormat {
    pub fn frame_bytes(&self) -> usize {
        if self.max_frame_size != 0 {
            return self.max_frame_size as usize;
        }
        let pixels = (self.width as usize).saturating_mul(self.height as usize);
        match self.format_type {
            VideoFormatType::Uncompressed(t) => {
                let pixel_size = match t {
                    UncompressedFormat::Yuy2 => 2,  // YUY2 每像素2字节
                    UncompressedFormat::Nv12 => 3,  // NV12 每两个像素平均3字节
                    UncompressedFormat::Rgb24 => 3, // RGB24 每像素3字节
                    UncompressedFormat::Rgb32 => 4, // RGB32 每像素4字节
                };
                let bytes = pixels.saturating_mul(pixel_size);
                if t == UncompressedFormat::Nv12 {
                    bytes / 2
                } else {
                    bytes
                }
            }
            VideoFormatType::Mjpeg => {
                // MJPEG 压缩后大小不定，这里返回一个估算值（假设压缩比为10:1）
                pixels.saturating_mul(3) / 10
            }
            VideoFormatType::H264 => {
                // H.264 压缩后大小不定，这里返回一个估算值（假设压缩比为20:1）
                pixels.saturating_mul(3) / 20
            }
        }
    }
}

/// 未压缩视频格式类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UncompressedFormat {
    /// YUY2 (YUYV) 格式
    Yuy2,
    /// NV12 格式
    Nv12,
    /// RGB24 格式
    Rgb24,
    /// RGB32 格式
    Rgb32,
}

/// 视频控制事件
#[derive(Debug, Clone)]
pub enum VideoControlEvent {
    /// 视频格式变更
    FormatChanged(VideoFormat),
    /// 亮度调整
    BrightnessChanged(i16),
    /// 对比度调整
    ContrastChanged(i16),
    /// 色调调整
    HueChanged(i16),
    /// 饱和度调整
    SaturationChanged(i16),
    /// 错误事件
    Error(String),
}

/// 视频数据帧
#[derive(Debug)]
pub struct VideoFrame {
    /// 帧数据
    pub data: Vec<u8>,
    /// UVC PTS in the 90 kHz clock domain; zero when the payload has no PTS.
    pub timestamp: u64,
    /// 帧序号
    pub frame_number: u32,
    /// 数据格式
    pub format: VideoFormat,
    /// 是否是帧结束标志
    pub end_of_frame: bool,
    /// The UVC payload stream reported an error while assembling this frame.
    pub has_error: bool,
}

/// UVC 设备状态
#[derive(Debug, Clone, PartialEq)]
pub enum UvcDeviceState {
    /// 未配置
    Unconfigured,
    /// 已配置但未开始流传输
    Configured,
    /// 正在进行流传输
    Streaming,
    /// 错误状态
    Error(String),
}

/// UVC Stream Control 结构体 (参考 UVC 规范 4.3.1.1)
#[derive(Debug, Clone)]
struct StreamControl {
    hint: u16,                      // bmHint
    format_index: u8,               // bFormatIndex
    frame_index: u8,                // bFrameIndex
    frame_interval: u32,            // dwFrameInterval (100ns units)
    key_frame_rate: u16,            // wKeyFrameRate
    p_frame_rate: u16,              // wPFrameRate
    comp_quality: u16,              // wCompQuality
    comp_window_size: u16,          // wCompWindowSize
    delay: u16,                     // wDelay
    max_video_frame_size: u32,      // dwMaxVideoFrameSize
    max_payload_transfer_size: u32, // dwMaxPayloadTransferSize
    clock_frequency: u32,           // UVC 1.1: dwClockFrequency
    framing_info: u8,               // UVC 1.1: bmFramingInfo
    preferred_version: u8,          // UVC 1.1: bPreferedVersion
    min_version: u8,                // UVC 1.1: bMinVersion
    max_version: u8,                // UVC 1.1: bMaxVersion
    usage: u8,                      // UVC 1.5: bUsage
    bit_depth: u8,                  // UVC 1.5: bBitDepthLuma
    settings: u8,                   // UVC 1.5: bmSettings
    max_ref_frames: u8,             // UVC 1.5: bMaxNumberOfRefFramesPlus1
    layout_per_stream: [u8; 10],    // UVC 1.5: bmLayoutPerStream
}

#[derive(Debug, Clone)]
struct VideoMode {
    format: VideoFormat,
    format_index: u8,
    frame_index: u8,
    frame_interval: u32,
    max_video_frame_size: u32,
}

#[derive(Debug, Default)]
struct ParsedCapabilities {
    modes: Vec<VideoMode>,
    processing_unit_id: Option<u8>,
    uvc_version: Option<u16>,
    has_video_control_header: bool,
    has_video_streaming_header: bool,
    streaming_endpoint_address: Option<u8>,
}

pub struct UvcDevice {
    device: Device,

    video_control_interface_num: u8,
    video_streaming_interface_num: u8,
    processing_unit_id: Option<u8>, // 处理单元ID
    current_format: Option<VideoFormat>,
    negotiated_control: Option<StreamControl>,
    video_modes: Option<Vec<VideoMode>>,
    uvc_version: u16,
    streaming_endpoint_address: Option<u8>,
    state: UvcDeviceState,
    descriptor_parser: DescriptorParser,
}

impl UvcDevice {
    /// 检查设备是否为 UVC 设备
    pub fn check(info: &DeviceInfo) -> bool {
        let mut has_video_control = false;
        let mut has_video_streaming = false;

        for iface in info.interface_descriptors() {
            match iface.class() {
                Class::Video | Class::AudioVideo(_) => {
                    // UVC Video Control Interface (subclass=1)
                    if iface.subclass == 1 {
                        has_video_control = true;
                    }
                    // UVC Video Streaming Interface (subclass=2)
                    if iface.subclass == 2 {
                        has_video_streaming = true;
                    }
                }
                _ => {}
            }
        }

        has_video_control && has_video_streaming
    }

    /// Creates a UVC device and selects its VideoControl and idle VideoStreaming interfaces.
    pub async fn new(mut device: Device) -> Result<Self, USBError> {
        let (video_control_interface_num, video_control_alt, video_streaming_interface_num) = {
            let config = device.configurations().first().ok_or(USBError::NotFound)?;
            let is_video_control = |interface: &usb_if::descriptor::InterfaceDescriptor| {
                matches!(interface.class(), Class::Video | Class::AudioVideo(_))
                    && interface.subclass == 1
            };
            let is_video_streaming = |interface: &usb_if::descriptor::InterfaceDescriptor| {
                matches!(interface.class(), Class::Video | Class::AudioVideo(_))
                    && interface.subclass == 2
            };

            let control = config
                .interfaces
                .iter()
                .flat_map(|group| group.alt_settings.iter())
                .find(|interface| interface.alternate_setting == 0 && is_video_control(interface))
                .ok_or(USBError::NotFound)?;
            let streaming = config
                .interfaces
                .iter()
                .flat_map(|group| group.alt_settings.iter())
                .find(|interface| interface.alternate_setting == 0 && is_video_streaming(interface))
                .ok_or(USBError::NotFound)?;
            (
                control.interface_number,
                control.alternate_setting,
                streaming.interface_number,
            )
        };

        device
            .claim_interface(video_control_interface_num, video_control_alt)
            .await?;
        // UVC devices are required to expose an idle VS alternate setting. An
        // explicit SET_INTERFACE is needed before Probe on some cameras.
        device
            .claim_interface(video_streaming_interface_num, 0)
            .await?;

        let mut uvc = Self {
            device,
            video_control_interface_num,
            video_streaming_interface_num,
            processing_unit_id: None,
            current_format: None,
            negotiated_control: None,
            video_modes: None,
            uvc_version: 0x0100,
            streaming_endpoint_address: None,
            state: UvcDeviceState::Configured,
            descriptor_parser: DescriptorParser::new(),
        };

        if let Some(raw) = uvc
            .device
            .configurations()
            .first()
            .map(|configuration| configuration.raw.clone())
            .filter(|raw| !raw.is_empty())
        {
            let capabilities = uvc.parse_capabilities(&raw)?;
            if !capabilities.modes.is_empty() {
                uvc.apply_capabilities(capabilities)?;
            } else {
                uvc.uvc_version = capabilities.uvc_version.unwrap_or(0x0100);
                uvc.processing_unit_id = capabilities.processing_unit_id;
                uvc.streaming_endpoint_address = capabilities.streaming_endpoint_address;
            }
        }

        Ok(uvc)
    }

    /// Returns the formats and frame intervals advertised by the VS descriptors.
    pub async fn get_supported_formats(&mut self) -> Result<Vec<VideoFormat>, USBError> {
        self.ensure_video_modes().await?;
        let modes = self.video_modes.as_ref().ok_or(USBError::NotSupported)?;
        Ok(modes.iter().map(|mode| mode.format.clone()).collect())
    }

    async fn ensure_video_modes(&mut self) -> Result<(), USBError> {
        if self.video_modes.is_some() {
            return Ok(());
        }
        let raw = self.get_full_configuration_descriptor().await?;
        let capabilities = self.parse_capabilities(&raw)?;
        self.apply_capabilities(capabilities)
    }

    fn apply_capabilities(&mut self, capabilities: ParsedCapabilities) -> Result<(), USBError> {
        if capabilities.modes.is_empty()
            || !capabilities.has_video_control_header
            || !capabilities.has_video_streaming_header
        {
            return Err(USBError::NotSupported);
        }
        if let Some(version) = capabilities.uvc_version {
            self.uvc_version = version;
        }
        self.processing_unit_id = capabilities.processing_unit_id;
        if capabilities.streaming_endpoint_address == Some(0) {
            return Err(USBError::InvalidParameter);
        }
        self.streaming_endpoint_address = capabilities.streaming_endpoint_address;
        self.video_modes = Some(capabilities.modes);
        Ok(())
    }

    /// 通过控制请求获取完整的配置描述符
    async fn get_full_configuration_descriptor(&mut self) -> Result<Vec<u8>, USBError> {
        let setup = ControlSetup {
            request_type: RequestType::Standard,
            recipient: Recipient::Device,
            request: Request::GetDescriptor,
            value: (0x02 << 8), // Configuration descriptor type
            index: 0,           // Configuration index
        };

        // 首先获取配置描述符头来确定总长度
        let mut header_buffer = vec![0u8; 9]; // 配置描述符头是9字节
        let header_length = self.device.control_in(setup, &mut header_buffer).await?;

        if header_length < 4 {
            Err(anyhow!("Failed to read configuration descriptor header"))?;
        }

        // 提取总长度（小端格式）
        let total_length = u16::from_le_bytes([header_buffer[2], header_buffer[3]]) as usize;
        trace!("Configuration descriptor total length: {total_length} bytes");

        if total_length < 9 {
            Err(anyhow!("Invalid configuration descriptor length"))?;
        }

        // 获取完整的配置描述符
        let mut full_buffer = alloc::vec![0u8; total_length];
        let setup_full = ControlSetup {
            request_type: RequestType::Standard,
            recipient: Recipient::Device,
            request: Request::GetDescriptor,
            value: (0x02 << 8), // Configuration descriptor type
            index: 0,           // Configuration index
        };

        let actual_length = self.device.control_in(setup_full, &mut full_buffer).await?;
        if actual_length < 9 {
            Err(anyhow!("Configuration descriptor response is too short"))?;
        }
        full_buffer.truncate(actual_length.min(full_buffer.len()));

        Ok(full_buffer)
    }

    fn parse_capabilities(&self, data: &[u8]) -> Result<ParsedCapabilities, USBError> {
        let mut capabilities = ParsedCapabilities::default();
        let mut current_interface = None;
        let mut current_format = None;
        let mut position = 0usize;

        while position < data.len() {
            if position + 2 > data.len() {
                return Err(USBError::InvalidParameter);
            }
            let length = usize::from(data[position]);
            if length < 2 || position + length > data.len() {
                return Err(USBError::InvalidParameter);
            }
            let descriptor = &data[position..position + length];

            match descriptor[1] {
                uvc_descriptor_types::INTERFACE if length >= 9 => {
                    let interface_number = descriptor[2];
                    let interface_class = descriptor[5];
                    let interface_subclass = descriptor[6];
                    current_interface =
                        (interface_class == 0x0e).then_some((interface_number, interface_subclass));
                    current_format = None;
                }
                uvc_descriptor_types::CS_INTERFACE if length >= 3 => {
                    let Some((interface_number, interface_subclass)) = current_interface else {
                        position += length;
                        continue;
                    };
                    let subtype = descriptor[2];
                    if interface_subclass == 1
                        && interface_number == self.video_control_interface_num
                    {
                        match subtype {
                            uvc_interface_subtypes::VC_HEADER => {
                                let header = self.descriptor_parser.parse_vc_header(descriptor)?;
                                capabilities.uvc_version = Some(header.bcd_uvc);
                                capabilities.has_video_control_header = true;
                            }
                            uvc_interface_subtypes::VC_PROCESSING_UNIT => {
                                let unit =
                                    self.descriptor_parser.parse_processing_unit(descriptor)?;
                                capabilities.processing_unit_id.get_or_insert(unit.unit_id);
                            }
                            _ => {}
                        }
                    } else if interface_subclass == 2
                        && interface_number == self.video_streaming_interface_num
                    {
                        match subtype {
                            uvc_interface_subtypes::VS_INPUT_HEADER => {
                                let header =
                                    self.descriptor_parser.parse_vs_input_header(descriptor)?;
                                capabilities.has_video_streaming_header = true;
                                capabilities.streaming_endpoint_address =
                                    Some(header.endpoint_address);
                            }
                            uvc_interface_subtypes::VS_FORMAT_UNCOMPRESSED if length >= 27 => {
                                let format = self
                                    .descriptor_parser
                                    .parse_uncompressed_format(descriptor)?;
                                let format_type = if format.guid == format_guids::YUY2 {
                                    Some(UncompressedFormat::Yuy2)
                                } else if format.guid == format_guids::NV12 {
                                    Some(UncompressedFormat::Nv12)
                                } else if format.guid == format_guids::RGB24 {
                                    Some(UncompressedFormat::Rgb24)
                                } else {
                                    None
                                };
                                current_format = format_type
                                    .map(VideoFormatType::Uncompressed)
                                    .map(|format_type| (format.format_index, format_type));
                            }
                            uvc_interface_subtypes::VS_FORMAT_MJPEG if length >= 11 => {
                                let format =
                                    self.descriptor_parser.parse_mjpeg_format(descriptor)?;
                                current_format =
                                    Some((format.format_index, VideoFormatType::Mjpeg));
                            }
                            uvc_interface_subtypes::VS_FORMAT_H264 if length >= 4 => {
                                current_format = Some((descriptor[3], VideoFormatType::H264));
                            }
                            uvc_interface_subtypes::VS_FRAME_UNCOMPRESSED
                            | uvc_interface_subtypes::VS_FRAME_MJPEG
                            | uvc_interface_subtypes::VS_FRAME_H264 => {
                                let Some((format_index, format_type)) = current_format else {
                                    position += length;
                                    continue;
                                };
                                let expected_subtype = match format_type {
                                    VideoFormatType::Uncompressed(_) => {
                                        uvc_interface_subtypes::VS_FRAME_UNCOMPRESSED
                                    }
                                    VideoFormatType::Mjpeg => {
                                        uvc_interface_subtypes::VS_FRAME_MJPEG
                                    }
                                    VideoFormatType::H264 => uvc_interface_subtypes::VS_FRAME_H264,
                                };
                                if subtype != expected_subtype {
                                    position += length;
                                    continue;
                                }
                                if format_index == 0 {
                                    return Err(USBError::InvalidParameter);
                                }
                                let frame =
                                    self.descriptor_parser.parse_frame_descriptor(descriptor)?;
                                if frame.frame_index == 0 {
                                    return Err(USBError::InvalidParameter);
                                }
                                let intervals = if frame.frame_interval_type == 0 {
                                    vec![frame.default_frame_interval]
                                } else {
                                    frame.frame_intervals.clone()
                                };
                                for interval in intervals {
                                    let frame_rate = DescriptorParser::interval_to_fps(interval);
                                    if interval == 0 || frame_rate == 0 {
                                        continue;
                                    }
                                    capabilities.modes.push(VideoMode {
                                        format: VideoFormat {
                                            width: frame.width,
                                            height: frame.height,
                                            frame_rate,
                                            format_type,
                                            max_frame_size: frame.max_video_frame_buffer_size,
                                        },
                                        format_index,
                                        frame_index: frame.frame_index,
                                        frame_interval: interval,
                                        max_video_frame_size: frame.max_video_frame_buffer_size,
                                    });
                                }
                            }
                            _ => {}
                        }
                    }
                }
                _ => {}
            }

            position += length;
        }

        Ok(capabilities)
    }

    /// 设置视频格式
    pub async fn set_format(&mut self, format: VideoFormat) -> Result<(), USBError> {
        self.ensure_video_modes().await?;
        let mode = self
            .video_modes
            .as_ref()
            .and_then(|modes| {
                modes.iter().find(|mode| {
                    mode.format == format
                        && mode.frame_interval
                            == DescriptorParser::fps_to_interval(format.frame_rate)
                })
            })
            .ok_or(USBError::InvalidParameter)?
            .clone();

        if mode.max_video_frame_size == 0 {
            return Err(USBError::NotSupported);
        }

        let mut probe = StreamControl {
            hint: 1,
            format_index: mode.format_index,
            frame_index: mode.frame_index,
            frame_interval: mode.frame_interval,
            key_frame_rate: 0,
            p_frame_rate: 0,
            comp_quality: 0,
            comp_window_size: 0,
            delay: 0,
            max_video_frame_size: mode.max_video_frame_size,
            max_payload_transfer_size: 0,
            clock_frequency: 0,
            framing_info: 0,
            preferred_version: 0,
            min_version: 0,
            max_version: 0,
            usage: 0,
            bit_depth: 0,
            settings: 0,
            max_ref_frames: 0,
            layout_per_stream: [0; 10],
        };

        info!(
            "[uvc-probe] request format={} frame={} interval={} advertised_frame_size={}",
            probe.format_index, probe.frame_index, probe.frame_interval, probe.max_video_frame_size,
        );

        self.send_vs_control(vs_controls::VS_PROBE_CONTROL, &probe)
            .await?;
        let min_comp_quality = self
            .get_probe_compression_quality(uvc_requests::GET_MIN)
            .await?;
        probe.comp_quality = self
            .get_probe_compression_quality(uvc_requests::GET_MAX)
            .await?;
        info!(
            "[uvc-probe] compression_quality min={} max={}",
            min_comp_quality, probe.comp_quality
        );

        // A second SET_CUR/GET_CUR cycle lets devices converge any dependent
        // payload and compression values before the Commit request.
        for round in 1..=2 {
            self.send_vs_control(vs_controls::VS_PROBE_CONTROL, &probe)
                .await?;
            let probe_response = self
                .get_vs_control(vs_controls::VS_PROBE_CONTROL, self.stream_control_size())
                .await?;
            probe = self.parse_stream_control(&probe_response)?;
            self.validate_probe_response(&probe, &mode)?;
            info!(
                "[uvc-probe] round={} response format={} frame={} interval={} frame_size={} \
                 payload_size={} comp_quality={}",
                round,
                probe.format_index,
                probe.frame_index,
                probe.frame_interval,
                probe.max_video_frame_size,
                probe.max_payload_transfer_size,
                probe.comp_quality,
            );
        }

        self.send_vs_control(vs_controls::VS_COMMIT_CONTROL, &probe)
            .await?;
        info!(
            "[uvc-probe] committed format={} frame={} interval={} frame_size={} payload_size={}",
            probe.format_index,
            probe.frame_index,
            probe.frame_interval,
            probe.max_video_frame_size,
            probe.max_payload_transfer_size,
        );
        self.current_format = Some(VideoFormat {
            frame_rate: DescriptorParser::interval_to_fps(probe.frame_interval),
            max_frame_size: probe.max_video_frame_size,
            ..format
        });
        self.negotiated_control = Some(probe);
        Ok(())
    }

    fn validate_probe_response(
        &self,
        probe: &StreamControl,
        mode: &VideoMode,
    ) -> Result<(), USBError> {
        if probe.format_index != mode.format_index
            || probe.frame_index != mode.frame_index
            || probe.frame_interval == 0
            || DescriptorParser::interval_to_fps(probe.frame_interval) == 0
            || probe.max_video_frame_size == 0
            || probe.max_payload_transfer_size == 0
        {
            return Err(USBError::NotSupported);
        }
        Ok(())
    }

    /// 开始视频流传输
    pub async fn start_streaming(&mut self) -> Result<VideoStream, USBError> {
        if self.state == UvcDeviceState::Streaming {
            return Err(USBError::InvalidParameter);
        }
        let vs_interface_num = self.video_streaming_interface_num;

        let current_format = self
            .current_format
            .clone()
            .ok_or(USBError::InvalidParameter)?;
        let negotiated = self
            .negotiated_control
            .clone()
            .ok_or(USBError::InvalidParameter)?;

        // 参考 libuvc 的实现，根据 dwMaxPayloadTransferSize 选择合适的 alternate setting
        let config = self
            .device
            .configurations()
            .first()
            .ok_or(USBError::NotFound)?;
        let vs_interface_group = config
            .interfaces
            .iter()
            .find(|iface| iface.interface_number == vs_interface_num)
            .ok_or(USBError::NotFound)?;

        let required_payload = usize::try_from(negotiated.max_payload_transfer_size)
            .map_err(|_| USBError::InvalidParameter)?;
        let streaming_endpoint_address = self
            .streaming_endpoint_address
            .ok_or(USBError::NotSupported)?;
        let mut selected = None;
        for alt_setting in &vs_interface_group.alt_settings {
            for endpoint in &alt_setting.endpoints {
                if endpoint.address != streaming_endpoint_address
                    || endpoint.direction != Direction::In
                    || !matches!(
                        endpoint.transfer_type,
                        EndpointType::Isochronous | EndpointType::Bulk
                    )
                {
                    continue;
                }
                let capacity = usize::from(endpoint.max_packet_size).saturating_mul(
                    if endpoint.transfer_type == EndpointType::Isochronous {
                        endpoint.packets_per_microframe.max(1)
                    } else {
                        1
                    },
                );
                if endpoint.transfer_type == EndpointType::Isochronous
                    && capacity < required_payload
                {
                    continue;
                }
                let replace = selected
                    .as_ref()
                    .is_none_or(|(_, _, current_capacity)| capacity < *current_capacity);
                if replace {
                    selected = Some((alt_setting.clone(), endpoint.clone(), capacity));
                }
            }
        }

        let (alt_setting, endpoint_desc, selected_capacity) =
            selected.ok_or(USBError::NotSupported)?;

        info!(
            "[uvc-stream] interface={} alt={} endpoint={:#04x} type={:?} max_packet={} \
             packets_per_microframe={} capacity={} negotiated_payload={}",
            vs_interface_num,
            alt_setting.alternate_setting,
            endpoint_desc.address,
            endpoint_desc.transfer_type,
            endpoint_desc.max_packet_size,
            endpoint_desc.packets_per_microframe,
            selected_capacity,
            required_payload,
        );

        // 切换到选中的 alternate setting
        self.device
            .claim_interface(vs_interface_num, alt_setting.alternate_setting)
            .await?;

        let ep = self.device.endpoint(endpoint_desc.address)?;
        let stream = VideoStream::new_with_transfer_size(
            ep,
            endpoint_desc,
            current_format,
            negotiated.max_payload_transfer_size,
            negotiated.max_video_frame_size,
        );
        self.state = UvcDeviceState::Streaming;
        Ok(stream)
    }

    /// Stops payload transfers and returns the VS interface to its idle setting.
    pub async fn stop_streaming(&mut self) -> Result<(), USBError> {
        if self.state != UvcDeviceState::Streaming {
            return Ok(());
        }
        let idle_alternate = self
            .device
            .configurations()
            .first()
            .and_then(|config| {
                config
                    .interfaces
                    .iter()
                    .find(|group| {
                        group.alt_settings.first().is_some_and(|interface| {
                            interface.interface_number == self.video_streaming_interface_num
                                && matches!(interface.class(), Class::Video | Class::AudioVideo(_))
                                && interface.subclass == 2
                        })
                    })
                    .and_then(|group| group.alt_settings.first())
                    .map(|interface| interface.alternate_setting)
            })
            .ok_or(USBError::NotFound)?;
        self.device
            .claim_interface(self.video_streaming_interface_num, idle_alternate)
            .await?;
        self.state = UvcDeviceState::Configured;
        Ok(())
    }

    /// 获取当前设备状态
    pub fn get_state(&self) -> &UvcDeviceState {
        &self.state
    }

    /// 获取当前视频格式
    pub fn get_current_format(&self) -> Option<&VideoFormat> {
        self.current_format.as_ref()
    }

    /// 发送视频控制命令
    pub async fn send_control_command(
        &mut self,
        command: VideoControlEvent,
    ) -> Result<(), USBError> {
        debug!("Sending video control command: {command:?}");

        let processing_unit_id = self.processing_unit_id.ok_or(USBError::NotFound)?;

        match command {
            VideoControlEvent::BrightnessChanged(value) => {
                debug!("Setting brightness to: {value}");
                self.send_pu_control(
                    pu_controls::PU_BRIGHTNESS_CONTROL,
                    processing_unit_id,
                    &value.to_le_bytes(),
                )
                .await?;
            }
            VideoControlEvent::ContrastChanged(value) => {
                debug!("Setting contrast to: {value}");
                self.send_pu_control(
                    pu_controls::PU_CONTRAST_CONTROL,
                    processing_unit_id,
                    &(value as u16).to_le_bytes(),
                )
                .await?;
            }
            VideoControlEvent::HueChanged(value) => {
                debug!("Setting hue to: {value}");
                self.send_pu_control(
                    pu_controls::PU_HUE_CONTROL,
                    processing_unit_id,
                    &value.to_le_bytes(),
                )
                .await?;
            }
            VideoControlEvent::SaturationChanged(value) => {
                debug!("Setting saturation to: {value}");
                self.send_pu_control(
                    pu_controls::PU_SATURATION_CONTROL,
                    processing_unit_id,
                    &(value as u16).to_le_bytes(),
                )
                .await?;
            }
            _ => {
                return Err(USBError::NotSupported);
            }
        }

        Ok(())
    }

    /// 发送处理单元控制请求
    async fn send_pu_control(
        &mut self,
        control_selector: u8,
        unit_id: u8,
        data: &[u8],
    ) -> Result<(), USBError> {
        let setup = ControlSetup {
            request_type: RequestType::Class,
            recipient: Recipient::Interface,
            request: uvc_requests::SET_CUR.into(),
            value: (control_selector as u16) << 8,
            index: (u16::from(self.video_control_interface_num) << 8) | u16::from(unit_id),
        };

        let written = self.device.control_out(setup, data).await?;
        if written != data.len() {
            return Err(USBError::TransferError(usb_if::err::TransferError::Other(
                anyhow!("short UVC processing-unit write: {written}/{}", data.len()),
            )));
        }

        Ok(())
    }

    /// 发送 VS 控制请求
    async fn send_vs_control(
        &mut self,
        control_selector: u8,
        stream_ctrl: &StreamControl,
    ) -> Result<(), USBError> {
        let vs_interface_num = self.video_streaming_interface_num;

        // 序列化 StreamControl 到字节数组
        let data = self.serialize_stream_control(stream_ctrl);

        let setup = ControlSetup {
            request_type: RequestType::Class,
            recipient: Recipient::Interface,
            request: uvc_requests::SET_CUR.into(),
            value: (control_selector as u16) << 8,
            index: vs_interface_num as u16,
        };

        debug!(
            "Sending VS control: selector=0x{:02x}, data_len={}",
            control_selector,
            data.len()
        );

        // 使用 video control 接口发送请求到 video streaming 接口
        let written = self.device.control_out(setup, &data).await?;
        if written != data.len() {
            return Err(USBError::TransferError(usb_if::err::TransferError::Other(
                anyhow!("short UVC stream-control write: {written}/{}", data.len()),
            )));
        }

        Ok(())
    }

    /// 获取 VS 控制响应
    async fn get_vs_control(
        &mut self,
        control_selector: u8,
        length: usize,
    ) -> Result<Vec<u8>, USBError> {
        self.get_vs_control_request(uvc_requests::GET_CUR, control_selector, length)
            .await
    }

    async fn get_vs_control_request(
        &mut self,
        request: u8,
        control_selector: u8,
        length: usize,
    ) -> Result<Vec<u8>, USBError> {
        let vs_interface_num = self.video_streaming_interface_num;

        let setup = ControlSetup {
            request_type: RequestType::Class,
            recipient: Recipient::Interface,
            request: request.into(),
            value: (control_selector as u16) << 8,
            index: vs_interface_num as u16,
        };

        let mut buffer = vec![0u8; length];
        let actual_length = self.device.control_in(setup, &mut buffer).await?;
        buffer.truncate(actual_length.min(buffer.len()));

        debug!(
            "Received VS control response: selector=0x{:02x}, data_len={}",
            control_selector,
            buffer.len()
        );

        Ok(buffer)
    }

    async fn get_probe_compression_quality(&mut self, request: u8) -> Result<u16, USBError> {
        let response = self
            .get_vs_control_request(
                request,
                vs_controls::VS_PROBE_CONTROL,
                self.stream_control_size(),
            )
            .await?;
        if response.len() == 2 {
            return Ok(u16::from_le_bytes([response[0], response[1]]));
        }
        Ok(self.parse_stream_control(&response)?.comp_quality)
    }

    /// 序列化 StreamControl 结构体
    fn serialize_stream_control(&self, ctrl: &StreamControl) -> Vec<u8> {
        let mut data = Vec::with_capacity(26);

        // bmHint (2 bytes)
        data.extend(&ctrl.hint.to_le_bytes());
        // bFormatIndex (1 byte)
        data.push(ctrl.format_index);
        // bFrameIndex (1 byte)
        data.push(ctrl.frame_index);
        // dwFrameInterval (4 bytes)
        data.extend(&ctrl.frame_interval.to_le_bytes());
        // wKeyFrameRate (2 bytes)
        data.extend(&ctrl.key_frame_rate.to_le_bytes());
        // wPFrameRate (2 bytes)
        data.extend(&ctrl.p_frame_rate.to_le_bytes());
        // wCompQuality (2 bytes)
        data.extend(&ctrl.comp_quality.to_le_bytes());
        // wCompWindowSize (2 bytes)
        data.extend(&ctrl.comp_window_size.to_le_bytes());
        // wDelay (2 bytes)
        data.extend(&ctrl.delay.to_le_bytes());
        // dwMaxVideoFrameSize (4 bytes)
        data.extend(&ctrl.max_video_frame_size.to_le_bytes());
        // dwMaxPayloadTransferSize (4 bytes)
        data.extend(&ctrl.max_payload_transfer_size.to_le_bytes());

        if self.stream_control_size() >= 34 {
            data.extend(&ctrl.clock_frequency.to_le_bytes());
            data.push(ctrl.framing_info);
            data.push(ctrl.preferred_version);
            data.push(ctrl.min_version);
            data.push(ctrl.max_version);
        }
        if self.stream_control_size() >= 48 {
            data.push(ctrl.usage);
            data.push(ctrl.bit_depth);
            data.push(ctrl.settings);
            data.push(ctrl.max_ref_frames);
            data.extend_from_slice(&ctrl.layout_per_stream);
        }

        data.resize(self.stream_control_size(), 0);

        debug!("Serialized stream control: {} bytes", data.len());
        data
    }

    fn stream_control_size(&self) -> usize {
        match self.uvc_version {
            0x0000..=0x010f => 26,
            0x0110..=0x014f => 34,
            _ => 48,
        }
    }

    /// 解析 StreamControl 响应
    fn parse_stream_control(&self, data: &[u8]) -> Result<StreamControl, USBError> {
        if data.len() < 26 {
            Err(anyhow!("Stream control response too short"))?;
        }

        let hint = u16::from_le_bytes([data[0], data[1]]);
        let format_index = data[2];
        let frame_index = data[3];
        let frame_interval = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        let key_frame_rate = u16::from_le_bytes([data[8], data[9]]);
        let p_frame_rate = u16::from_le_bytes([data[10], data[11]]);
        let comp_quality = u16::from_le_bytes([data[12], data[13]]);
        let comp_window_size = u16::from_le_bytes([data[14], data[15]]);
        let delay = u16::from_le_bytes([data[16], data[17]]);
        let max_video_frame_size = u32::from_le_bytes([data[18], data[19], data[20], data[21]]);
        let max_payload_transfer_size =
            u32::from_le_bytes([data[22], data[23], data[24], data[25]]);
        let clock_frequency = data
            .get(26..30)
            .map(|bytes| u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
            .unwrap_or(0);
        let framing_info = data.get(30).copied().unwrap_or(0);
        let preferred_version = data.get(31).copied().unwrap_or(0);
        let min_version = data.get(32).copied().unwrap_or(0);
        let max_version = data.get(33).copied().unwrap_or(0);
        let usage = data.get(34).copied().unwrap_or(0);
        let bit_depth = data.get(35).copied().unwrap_or(0);
        let settings = data.get(36).copied().unwrap_or(0);
        let max_ref_frames = data.get(37).copied().unwrap_or(0);
        let mut layout_per_stream = [0; 10];
        if let Some(layout) = data.get(38..48) {
            layout_per_stream.copy_from_slice(layout);
        }

        debug!(
            "Parsed stream control: format={format_index}, frame={frame_index}, \
             interval={frame_interval}, max_frame_size={max_video_frame_size}"
        );

        Ok(StreamControl {
            hint,
            format_index,
            frame_index,
            frame_interval,
            key_frame_rate,
            p_frame_rate,
            comp_quality,
            comp_window_size,
            delay,
            max_video_frame_size,
            max_payload_transfer_size,
            clock_frequency,
            framing_info,
            preferred_version,
            min_version,
            max_version,
            usage,
            bit_depth,
            settings,
            max_ref_frames,
            layout_per_stream,
        })
    }

    // /// 获取当前的 Stream Control 参数
    // async fn get_current_stream_control(&mut self) -> Result<StreamControl, USBError> {
    //     // 发送 GET_CUR 请求获取当前的 commit 参数
    //     debug!("Getting current stream control parameters");
    //     let response = self
    //         .get_vs_control(vs_controls::VS_COMMIT_CONTROL, 26)
    //         .await?;
    //     self.parse_stream_control(&response)
    // }

    /// 获取设备信息字符串
    pub async fn get_device_info(&self) -> Result<String, USBError> {
        Ok(format!(
            "UVC {:04x}:{:04x}",
            self.device.vendor_id(),
            self.device.product_id()
        ))
    }

    /// 获取流错误代码
    pub async fn get_stream_error_code(&mut self) -> Result<u8, USBError> {
        debug!("Getting stream error code");
        let response = self
            .get_vs_control(vs_controls::STREAM_ERROR_CODE, 1)
            .await?;
        let error_code = response
            .first()
            .copied()
            .ok_or(USBError::InvalidParameter)?;
        debug!("Stream error code: 0x{:02x}", error_code);
        Ok(error_code)
    }
}
