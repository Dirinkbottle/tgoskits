#![cfg(not(target_os = "none"))]

use crab_uvc::{
    UncompressedFormat, UvcDeviceState, VideoControlEvent, VideoFormat, VideoFormatType,
};

#[test]
fn test_video_format_creation() {
    let mjpeg_format = VideoFormat {
        width: 1920,
        height: 1080,
        frame_rate: 30,
        format_type: VideoFormatType::Mjpeg,
        max_frame_size: 1_048_576,
    };

    assert_eq!(mjpeg_format.width, 1920);
    assert_eq!(mjpeg_format.height, 1080);
    assert_eq!(mjpeg_format.frame_rate, 30);
    assert!(matches!(mjpeg_format.format_type, VideoFormatType::Mjpeg));
}

#[test]
fn test_uncompressed_format_creation() {
    let yuy2_format = VideoFormat {
        width: 640,
        height: 480,
        frame_rate: 30,
        format_type: VideoFormatType::Uncompressed(UncompressedFormat::Yuy2),
        max_frame_size: 614_400,
    };

    assert_eq!(yuy2_format.width, 640);
    assert_eq!(yuy2_format.height, 480);
    assert_eq!(yuy2_format.frame_rate, 30);
    assert_eq!(
        yuy2_format.format_type,
        VideoFormatType::Uncompressed(UncompressedFormat::Yuy2)
    );
}

#[test]
fn test_video_control_events() {
    let brightness_event = VideoControlEvent::BrightnessChanged(100);
    match brightness_event {
        VideoControlEvent::BrightnessChanged(value) => {
            assert_eq!(value, 100);
        }
        _ => panic!("Unexpected event type"),
    }

    let contrast_event = VideoControlEvent::ContrastChanged(50);
    match contrast_event {
        VideoControlEvent::ContrastChanged(value) => {
            assert_eq!(value, 50);
        }
        _ => panic!("Unexpected event type"),
    }
}

#[test]
fn test_device_states() {
    let state = UvcDeviceState::Unconfigured;
    assert_eq!(state, UvcDeviceState::Unconfigured);

    let configured_state = UvcDeviceState::Configured;
    assert_eq!(configured_state, UvcDeviceState::Configured);

    let streaming_state = UvcDeviceState::Streaming;
    assert_eq!(streaming_state, UvcDeviceState::Streaming);

    let error_state = UvcDeviceState::Error("Test error".to_string());
    match error_state {
        UvcDeviceState::Error(msg) => {
            assert_eq!(msg, "Test error");
        }
        _ => panic!("Unexpected state type"),
    }
}

#[test]
fn test_format_equality() {
    let format1 = VideoFormat {
        width: 640,
        height: 480,
        frame_rate: 30,
        format_type: VideoFormatType::Mjpeg,
        max_frame_size: 262_144,
    };

    let format2 = VideoFormat {
        width: 640,
        height: 480,
        frame_rate: 30,
        format_type: VideoFormatType::Mjpeg,
        max_frame_size: 262_144,
    };

    assert_eq!(format1, format2);

    let format3 = VideoFormat {
        width: 1280,
        height: 720,
        frame_rate: 30,
        format_type: VideoFormatType::Mjpeg,
        max_frame_size: 524_288,
    };

    assert_ne!(format1, format3);
}

#[test]
fn test_uncompressed_format_types() {
    let formats = vec![
        UncompressedFormat::Yuy2,
        UncompressedFormat::Nv12,
        UncompressedFormat::Rgb24,
        UncompressedFormat::Rgb32,
    ];

    assert_eq!(formats.len(), 4);
    assert!(formats.contains(&UncompressedFormat::Yuy2));
    assert!(formats.contains(&UncompressedFormat::Nv12));
    assert!(formats.contains(&UncompressedFormat::Rgb24));
    assert!(formats.contains(&UncompressedFormat::Rgb32));
}
