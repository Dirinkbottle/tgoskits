//! UVC camera binding and the asynchronous frame-capture interface.

use alloc::{collections::VecDeque, vec::Vec};

use crab_usb::{
    DeviceInfo, USBHost,
    driver::{UsbDriver, UsbId, UsbInterface},
    usb_if::err::USBError,
};
use crab_uvc::{UvcDevice, UvcDeviceState, VideoFormat, VideoFrame, VideoStream};

struct UvcVideoDriver;

static UVCVIDEO_DRIVER: UvcVideoDriver = UvcVideoDriver;

static UVCVIDEO_IDS: &[UsbId] = &[
    UsbId {
        vid: None,
        pid: None,
        class: Some(0x0e),
        subclass: Some(0x01),
        protocol: None,
    },
    UsbId {
        vid: None,
        pid: None,
        class: Some(0x0e),
        subclass: Some(0x02),
        protocol: None,
    },
];

pub(super) fn register(host: &mut USBHost) {
    host.register_usb_driver(&UVCVIDEO_DRIVER, UVCVIDEO_IDS);
}

impl UsbDriver for UvcVideoDriver {
    fn name(&self) -> &'static str {
        "uvcvideo"
    }

    fn probe(&self, interface: &UsbInterface) -> Result<(), USBError> {
        if interface.class() != 0x0e || !matches!(interface.subclass(), 0x01 | 0x02) {
            return Err(USBError::NotSupported);
        }
        log::info!(
            "uvcvideo: recognized {:04x}:{:04x} interface {} alt {}",
            interface.device().vendor_id(),
            interface.device().product_id(),
            interface.interface_number(),
            interface.alternate_setting(),
        );
        Ok(())
    }
}

/// An opened UVC camera with a negotiated streaming format.
pub struct UvcVideoCapture {
    device: UvcDevice,
    stream: Option<VideoStream>,
    format: Option<VideoFormat>,
    pending_frames: VecDeque<VideoFrame>,
}

impl UvcVideoCapture {
    /// Opens a probed UVC device and claims its VideoControl interface.
    pub async fn open(host: &mut USBHost, info: &DeviceInfo) -> Result<Self, USBError> {
        if !UvcDevice::check(info) {
            return Err(USBError::NotSupported);
        }
        let device = host.open_device(info).await?;
        Ok(Self {
            device: UvcDevice::new(device).await?,
            stream: None,
            format: None,
            pending_frames: VecDeque::new(),
        })
    }

    /// Returns formats and frame rates explicitly advertised by the camera.
    pub async fn supported_formats(&mut self) -> Result<Vec<VideoFormat>, USBError> {
        self.device.get_supported_formats().await
    }

    /// Performs the UVC Probe/Commit exchange for `format`.
    pub async fn set_format(&mut self, format: VideoFormat) -> Result<(), USBError> {
        if self.stream.is_some() {
            return Err(USBError::InvalidParameter);
        }
        self.device.set_format(format.clone()).await?;
        self.format = Some(
            self.device
                .get_current_format()
                .cloned()
                .ok_or(USBError::InvalidParameter)?,
        );
        Ok(())
    }

    /// Starts streaming after selecting the requested advertised format.
    pub async fn start_streaming(&mut self, format: VideoFormat) -> Result<(), USBError> {
        self.set_format(format).await?;
        self.stream = Some(self.device.start_streaming().await?);
        Ok(())
    }

    /// Selects the first descriptor-advertised mode and starts streaming it.
    pub async fn start_default_streaming(&mut self) -> Result<(), USBError> {
        let format = self
            .supported_formats()
            .await?
            .into_iter()
            .next()
            .ok_or(USBError::NotSupported)?;
        self.start_streaming(format).await
    }

    /// Waits until one complete UVC frame has been assembled.
    pub async fn recv_frame(&mut self) -> Result<VideoFrame, USBError> {
        loop {
            if let Some(frame) = self.pending_frames.pop_front() {
                return Ok(frame);
            }

            let format = self.format.clone().ok_or(USBError::InvalidParameter)?;
            let stream = self.stream.as_mut().ok_or(USBError::InvalidParameter)?;
            let events = stream.recv().await?;
            for event in events {
                self.pending_frames.push_back(VideoFrame {
                    data: event.data,
                    timestamp: event.pts_90khz.map(u64::from).unwrap_or(0),
                    frame_number: event.frame_number,
                    format: format.clone(),
                    end_of_frame: event.eof,
                    has_error: event.has_error,
                });
            }
        }
    }

    /// Stops payload transfers and releases the active stream endpoint.
    pub async fn stop_streaming(&mut self) -> Result<(), USBError> {
        self.device.stop_streaming().await?;
        if let Some(stream) = self.stream.take() {
            stream.report_diagnostics();
        }
        self.pending_frames.clear();
        Ok(())
    }

    /// Returns the format accepted by the most recent UVC Probe/Commit exchange.
    pub fn current_format(&self) -> Option<&VideoFormat> {
        self.format.as_ref()
    }

    /// Returns the UVC state of the underlying device.
    pub fn state(&self) -> &UvcDeviceState {
        self.device.get_state()
    }
}

/// Opens the first complete UVC camera retained in the current USB topology.
pub async fn probe_first(host: &mut USBHost) -> Result<Option<UvcVideoCapture>, USBError> {
    host.probe_changes().await?;
    let camera = host
        .connected_devices()
        .find(|info| UvcDevice::check(info))
        .cloned();

    let Some(camera) = camera else {
        return Ok(None);
    };
    Ok(Some(UvcVideoCapture::open(host, &camera).await?))
}

/// Opens each complete UVC camera retained in the current USB topology.
pub async fn probe(host: &mut USBHost) -> Result<Vec<UvcVideoCapture>, USBError> {
    host.probe_changes().await?;
    let cameras = host
        .connected_devices()
        .filter(|info| UvcDevice::check(info))
        .cloned()
        .collect::<Vec<_>>();

    let mut captures = Vec::with_capacity(cameras.len());
    for info in cameras {
        captures.push(UvcVideoCapture::open(host, &info).await?);
    }
    Ok(captures)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registers_wildcard_video_control_and_streaming_interfaces() {
        assert_eq!(UVCVIDEO_DRIVER.name(), "uvcvideo");
        assert_eq!(UVCVIDEO_IDS.len(), 2);
        assert_eq!(UVCVIDEO_IDS[0].vid, None);
        assert_eq!(UVCVIDEO_IDS[0].pid, None);
        assert_eq!(UVCVIDEO_IDS[0].class, Some(0x0e));
        assert_eq!(UVCVIDEO_IDS[0].subclass, Some(0x01));
        assert_eq!(UVCVIDEO_IDS[1].vid, None);
        assert_eq!(UVCVIDEO_IDS[1].pid, None);
        assert_eq!(UVCVIDEO_IDS[1].class, Some(0x0e));
        assert_eq!(UVCVIDEO_IDS[1].subclass, Some(0x02));
    }
}
