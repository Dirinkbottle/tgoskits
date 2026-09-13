use alloc::vec::Vec;
use core::fmt::Debug;

use log::debug;
use usb_if::err::TransferError;

use crate::descriptors::payload_header_flags as flags;

/// UVC 载荷头（2.4.3.3）
#[derive(Debug, Clone, Default)]
pub struct UvcPayloadHeader {
    pub length: u8,              // bLength
    pub info: u8,                // bmHeaderInfo
    pub fid: bool,               // Frame ID
    pub eof: bool,               // End of Frame
    pub pts: Option<u32>,        // Presentation Time Stamp (4 bytes, 90kHz)
    pub scr: Option<(u32, u16)>, // Source Clock Reference: SOF timestamp (32) + SOF count (16)
    pub has_err: bool,
}

impl UvcPayloadHeader {
    /// 从字节流解析 UVC 载荷头；若数据不合法，返回 None 以允许上层丢弃该包。
    pub fn parse(buf: &[u8]) -> Option<(Self, usize)> {
        if buf.len() < 2 {
            return None;
        }
        let b_length = buf[0] as usize;
        let info = buf[1];
        if b_length < 2 || b_length > buf.len() {
            return None;
        }

        let fid = (info & flags::FID) != 0;
        let eof = (info & flags::EOF) != 0;
        let has_pts = (info & flags::PTS) != 0;
        let has_scr = (info & flags::SCR) != 0;
        let has_err = (info & flags::ERR) != 0;

        // 可选字段顺序：PTS(4) -> SCR(6)
        let mut _offset = 2usize;
        let pts = if has_pts {
            if _offset + 4 > b_length {
                return None;
            }
            let v = u32::from_le_bytes([
                buf[_offset],
                buf[_offset + 1],
                buf[_offset + 2],
                buf[_offset + 3],
            ]);
            _offset += 4;
            Some(v)
        } else {
            None
        };

        let scr = if has_scr {
            if _offset + 6 > b_length {
                return None;
            }
            let stc = u32::from_le_bytes([
                buf[_offset],
                buf[_offset + 1],
                buf[_offset + 2],
                buf[_offset + 3],
            ]);
            let sof = u16::from_le_bytes([buf[_offset + 4], buf[_offset + 5]]);
            _offset += 6;
            Some((stc, sof))
        } else {
            None
        };

        // 剩余可忽略的扩展字段由 b_length 统一跳过
        let header = UvcPayloadHeader {
            length: b_length as u8,
            info,
            fid,
            eof,
            pts,
            scr,
            has_err,
        };

        Some((header, b_length))
    }
}

/// 帧组装事件（供上层转换为具体视频帧结构）
#[derive(Debug, Clone)]
pub struct FrameEvent {
    pub data: Vec<u8>,
    pub pts_90khz: Option<u32>,
    pub eof: bool,
    pub fid: bool,
    pub frame_number: u32,
    pub has_error: bool,
}

/// UVC 帧解析/组装器（参考 libuvc 的 FID 翻转与 EOF 逻辑）
#[derive(Debug)]
pub struct FrameParser {
    buffer: Vec<u8>,
    last_fid: Option<bool>,
    last_pts: Option<u32>,
    frame_number: u32,
    error_packet_count: u32, // 统计错误包数量
    invalid_header_count: u32,
    frame_has_error: bool,
    frame_size: usize,
    /// The first payload after stream start may belong to a frame that began
    /// before the host submitted its first transfer. Do not publish it.
    synchronized: bool,
}

impl FrameParser {
    pub fn new(frame_size: usize) -> Self {
        Self {
            buffer: Vec::with_capacity(frame_size),
            last_fid: None,
            frame_number: 0,
            last_pts: None,
            error_packet_count: 0,
            invalid_header_count: 0,
            frame_has_error: false,
            frame_size,
            synchronized: false,
        }
    }

    /// Updates the Frame ID state and reports whether this packet starts a new frame.
    fn update_fid(&mut self, fid: bool) -> bool {
        let Some(last) = self.last_fid else {
            self.last_fid = Some(fid);
            return false;
        };

        if last == fid {
            return false;
        }

        debug!("FID toggled ({last} -> {fid})",);

        self.last_fid = Some(fid);

        self.buffer.clear();
        self.last_pts = None;
        self.frame_has_error = false;
        true
    }

    /// 获取错误包统计信息
    pub fn error_packet_count(&self) -> u32 {
        self.error_packet_count
    }

    pub(crate) fn invalid_header_count(&self) -> u32 {
        self.invalid_header_count
    }

    /// 重置错误包统计
    pub fn reset_error_count(&mut self) {
        self.error_packet_count = 0;
    }

    /// 处理一包 UVC 传输数据；返回完整帧事件（若 EOF 收到）
    pub fn push_packet(&mut self, data: &[u8]) -> Result<Option<FrameEvent>, TransferError> {
        if data.len() < 2 {
            return Ok(None);
        }

        let (hdr, hdr_len) = match UvcPayloadHeader::parse(data) {
            Some(v) => v,
            None => {
                self.invalid_header_count = self.invalid_header_count.wrapping_add(1);
                if self.invalid_header_count <= 4 || self.invalid_header_count.is_multiple_of(128) {
                    debug!(
                        "[uvc-frame] invalid payload header: packet_bytes={} \
                         total_invalid_headers={}",
                        data.len(),
                        self.invalid_header_count
                    );
                }
                return Ok(None);
            }
        };
        let starts_new_frame = self.update_fid(hdr.fid);
        if starts_new_frame {
            self.synchronized = true;
        }

        if !self.synchronized {
            // The first observed transfer can start in the middle of a frame.
            // Its bytes must not be exposed as a complete image. An EOF also
            // supplies a boundary for cameras that do not toggle FID reliably.
            if hdr.eof {
                self.buffer.clear();
                self.last_pts = None;
                self.frame_has_error = false;
                self.synchronized = true;
                debug!("[uvc-frame] synchronized at initial EOF boundary");
            }
            return Ok(None);
        }

        if hdr.has_err {
            // 记录统计信息，了解错误频率
            self.error_packet_count += 1;
            debug!(
                "UVC payload ERR set; marking current frame ({} bytes), total error packets: {}",
                self.buffer.len(),
                self.error_packet_count
            );
            debug!(
                "Error details: FID={}, EOF={}, PTS={:?}, SCR={:?}, info=0x{:02x}",
                hdr.fid, hdr.eof, hdr.pts, hdr.scr, hdr.info
            );

            // UVC 载荷头中的 ERR 标志表示设备端检测到错误，常见原因包括：
            // 1. 带宽不足：USB 总线带宽不够，导致数据传输延迟或丢失
            // 2. 设备内部错误：传感器或编码器出现临时故障
            // 3. 时序问题：主机请求数据的时机与设备生成数据的时机不匹配
            // 4. 缓冲区溢出：设备内部缓冲区满了，无法继续接收数据

            // 分析错误模式
            if self.error_packet_count % 32 == 1 {
                debug!(
                    "UVC error pattern analysis: {} errors so far, current PTS={:?}, last good \
                     PTS={:?}",
                    self.error_packet_count, hdr.pts, self.last_pts
                );
            }

            self.frame_has_error = true;
            if hdr.eof {
                return Ok(self.finish_frame(&hdr));
            }
            return Ok(None);
        }

        // 载荷数据在头之后
        if hdr_len <= data.len() {
            // 负载长度来自传输完成长度，而不是内容是否为零。零字节是合法视频数据。
            self.buffer.extend_from_slice(&data[hdr_len..]);
        }
        if let Some(pts) = hdr.pts {
            self.last_pts = Some(pts);
        }

        if hdr.eof {
            return Ok(self.finish_frame(&hdr));
        }

        Ok(None)
    }

    fn finish_frame(&mut self, hdr: &UvcPayloadHeader) -> Option<FrameEvent> {
        if self.buffer.is_empty() && !self.frame_has_error {
            return None;
        }
        let data = core::mem::replace(&mut self.buffer, Vec::with_capacity(self.frame_size));
        let event = FrameEvent {
            data,
            pts_90khz: self.last_pts.take(),
            eof: true,
            fid: hdr.fid,
            frame_number: self.frame_number,
            has_error: self.frame_has_error,
        };
        if event.frame_number < 4 || event.has_error {
            debug!(
                "[uvc-frame] complete frame={} fid={} bytes={} error={} pts={:?}",
                event.frame_number,
                event.fid,
                event.data.len(),
                event.has_error,
                event.pts_90khz,
            );
        }
        self.frame_number = self.frame_number.wrapping_add(1);
        self.frame_has_error = false;
        Some(event)
    }
}

#[cfg(test)]
mod tests {
    use super::FrameParser;

    #[test]
    fn discards_initial_partial_frame_before_publishing_a_complete_frame() {
        let mut parser = FrameParser::new(2);

        assert!(
            parser
                .push_packet(&[2, 0x00, 0x12])
                .expect("valid partial payload packet")
                .is_none()
        );
        assert!(
            parser
                .push_packet(&[2, 0x02, 0x34])
                .expect("valid partial EOF packet")
                .is_none()
        );

        assert!(
            parser
                .push_packet(&[2, 0x01, 0xff, 0xd8])
                .expect("valid frame-start packet")
                .is_none()
        );
        let event = parser
            .push_packet(&[2, 0x03, 0x7f, 0x00])
            .expect("valid payload packet")
            .expect("EOF must complete the frame");

        assert_eq!(event.data, [0xff, 0xd8, 0x7f, 0x00]);
        assert!(event.eof);
    }

    #[test]
    fn marks_a_completed_frame_when_a_payload_reports_an_error() {
        let mut parser = FrameParser::new(2);

        assert!(
            parser
                .push_packet(&[2, 0x02])
                .expect("initial EOF synchronizes the parser")
                .is_none()
        );
        assert!(
            parser
                .push_packet(&[2, 0x01, 0xff, 0xd8])
                .expect("valid frame payload")
                .is_none()
        );
        assert!(
            parser
                .push_packet(&[2, 0x41])
                .expect("ERR payload is retained as frame metadata")
                .is_none()
        );
        let event = parser
            .push_packet(&[2, 0x03, 0xff, 0xd9])
            .expect("valid EOF payload")
            .expect("EOF completes the errored frame");

        assert!(event.has_error);
        assert_eq!(event.data, [0xff, 0xd8, 0xff, 0xd9]);
        assert_eq!(parser.error_packet_count(), 1);
    }
}
