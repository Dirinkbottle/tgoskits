//! K3 MFPR（多功能引脚寄存器）位定义 + IO 电源域常量 + 驱动强度查表。
//!
//! 所有数值与命名严格对照 Linux `drivers/pinctrl/spacemit/pinctrl-k1.c`（K3 路径）：
//! - 位布局：pinctrl-k1.c:32-62（K3 注释块 + PAD_*_K3 宏）
//! - pin → 寄存器偏移：pinctrl-k1.c:196-201（`spacemit_k3_pin_to_offset`）
//! - pin → io_pd 偏移：pinctrl-k1.c:225-251（`spacemit_k3_pin_to_io_pd_offset`）
//! - IO 电源域解锁序列：pinctrl-k1.c:479-509（`spacemit_set_io_pwr_domain`）
//! - 驱动强度查表：pinctrl-k1.c:346-394（K3 专用 16 项表 + `spacemit_get_ds_value`）

// ============================================================================
// MFPR 位布局（pinctrl-k1.c:41-62）。K1/K3 共用低 8 位 + 高 pull 位，
// 仅 drive / schmitt 两个字段的宽度在 K3 上不同：drive=4bits、schmitt=1bit。
// ============================================================================
//
// K3 MFPR 位布局（pinctrl-k1.c:53-62）：
//   bit15     bit14    bit13    bit12:9     bit8       bit7          bit6:4    bit3          bit2:0
//   PULL_EN   PULLUP   PULLDN   DRIVE_K3    SCHMITT    SLEW_RATE_EN  EDGE      STRONG_PULL   MUX

/// 引脚复用功能选择（pinctrl-k1.c:41）。3 bits，Function 0-7。
pub const PAD_MUX: u32 = 0b111; // GENMASK(2, 0)

/// 强上拉使能（pinctrl-k1.c:42）。仅在 `bias-pull-up = <1>` 时置（上游
/// pinctrl-k1.c `PIN_CONFIG_BIAS_PULL_UP` 的 `if (arg == 1) v |= PAD_STRONG_PULL`）。
///
/// rdif 的 `Bias::PullUp` 无字段携带 arg，无法区分普通/强上拉，故 parser 把
/// 带 arg 的 `bias-pull-up` 翻译成 `K3_PIN_CONFIG_STRONG_PULL` vendor config，
/// 在 `apply_config` 里补设此位。
pub const PAD_STRONG_PULL: u32 = 1 << 3; // BIT(3)

/// 边沿检测上升沿（pinctrl-k1.c:43-45，EDGE_*）；pinctrl 驱动未在 config 路径用。
#[allow(dead_code)]
pub const PAD_EDGE_RISE: u32 = 1 << 4;
#[allow(dead_code)]
pub const PAD_EDGE_FALL: u32 = 1 << 5;
#[allow(dead_code)]
pub const PAD_EDGE_CLEAR: u32 = 1 << 6;

/// 压摆率使能（pinctrl-k1.c:46）。
#[allow(dead_code)]
pub const PAD_SLEW_RATE_EN: u32 = 1 << 7;

/// 施密特触发器（K3 单 bit，pinctrl-k1.c:61）。K1 是 GENMASK(9,8)（2 bits）。
pub const PAD_SCHMITT_K3: u32 = 1 << 8; // BIT(8)
/// `__ffs(PAD_SCHMITT_K3)`：arg 左移到此位（pinctrl-k1.c:741）。
///
/// K3 schmitt 是单 bit，arg 仅 0/1，故实际写入用 `if arg != 0 { PAD_SCHMITT_K3 }`，
/// 此 SHIFT 常量仅供文档对照。
#[allow(dead_code)]
pub const PAD_SCHMITT_K3_SHIFT: u32 = 8;

/// 驱动强度（K3 4 bits，pinctrl-k1.c:62）。K1 是 GENMASK(12,10)（3 bits）。
pub const PAD_DRIVE_K3: u32 = 0b1111 << 9; // GENMASK(12, 9)
/// `__ffs(PAD_DRIVE_K3)`：查表所得 val 左移到此位（pinctrl-k1.c:772）。
pub const PAD_DRIVE_K3_SHIFT: u32 = 9;

pub const PAD_PULLDOWN: u32 = 1 << 13; // BIT(13)，pinctrl-k1.c:49
pub const PAD_PULLUP: u32 = 1 << 14; // BIT(14)，pinctrl-k1.c:50
pub const PAD_PULL_EN: u32 = 1 << 15; // BIT(15)，pinctrl-k1.c:51

// ============================================================================
// IO 电源域（pinctrl-k1.c:64-81, 479-509）
// ============================================================================

/// io_pd 寄存器中 1.8V 使能位（pinctrl-k1.c:75）。置 1 → 1V8，清 0 → 3V3。
pub const IO_PWR_DOMAIN_V18EN: u32 = 1 << 2; // BIT(2)

/// ASFAR：AIB Secure Access First Address Register（pinctrl-k1.c:77）。相对 apbc + asar_offset。
pub const APBC_ASFAR: u32 = 0x00;
/// ASSAR：AIB Secure Access Second Address Register（pinctrl-k1.c:78）。相对 apbc + asar_offset。
pub const APBC_ASSAR: u32 = 0x04;

/// ASFAR 一次性解锁魔钥（pinctrl-k1.c:80）。
pub const APBC_ASFAR_AKEY: u32 = 0xbaba;
/// ASSAR 一次性解锁魔钥（pinctrl-k1.c:81）。
pub const APBC_ASSAR_AKEY: u32 = 0xeb10;

/// K3 IO 电源域寄存器偏移（pinctrl-k1.c:70-73）。
///
/// 每个 GPIO bank 共享一个 io_pd 寄存器——`pin_to_io_pd_offset` 把 pin 映射到
/// 其所属 bank 的 io_pd 寄存器偏移（相对 regs[1] = 0xd401e800 基址）。
mod pd_offset {
    pub const GPIO1_K3: u32 = 0x04;
    pub const GPIO2_KX: u32 = 0x0c;
    pub const GPIO4_K3: u32 = 0x20;
    pub const GPIO5_K3: u32 = 0x10;
    pub const MMC_KX: u32 = 0x1c;
    pub const QSPI_K3: u32 = 0x2c;
}

// ============================================================================
// pin → 寄存器偏移映射
// ============================================================================

/// K3 pin → MFPR 寄存器字节偏移（pinctrl-k1.c:196-201）。
///
/// K3 是近似线性映射：`pin << 2`，但 pin > 130 时跳过 2 个索引（pin += 2 后再 << 2）。
/// GMAC/UART/I2C 等常用引脚（pin 0-130）不受此修正影响，直接 `pin * 4`。
pub fn pin_to_offset(pin: u32) -> u32 {
    let pin = if pin > 130 { pin + 2 } else { pin };
    pin << 2
}

/// K3 pin → io_pd 寄存器偏移（pinctrl-k1.c:225-251）。
///
/// 返回 0 表示该 pin 无独立 IO 电源域寄存器（fixed-voltage 或 reserved pin）。
/// GMAC 引脚（pin 21-37）全部落在 GPIO2_Kx（0x0c）。
pub fn pin_to_io_pd_offset(pin: u32) -> u32 {
    use pd_offset::*;
    match pin {
        0..=20 => GPIO1_K3,
        21..=41 => GPIO2_KX,
        76..=98 => GPIO4_K3,
        99..=127 => GPIO5_K3,
        132..=137 => MMC_KX,
        138..=144 => QSPI_K3,
        _ => 0,
    }
}

// ============================================================================
// 驱动强度查表（pinctrl-k1.c:346-382，K3 专用，原样照抄，勿排序）
// ============================================================================

/// K3 1.8V 驱动强度表（pinctrl-k1.c:346-363）。元素 = `(寄存器 val, 目标 mA)`。
///
/// 注意 mA 序列非单调（第 7 项 14mA → 第 8 项 21mA 跳变）——这是硬件编码顺序，
/// 必须原样保留，查找时线性扫描取首个 mA ≥ 目标。
pub const K3_DS_1V8: [(u32, u32); 16] = [
    (0, 2),
    (1, 4),
    (2, 6),
    (3, 7),
    (4, 9),
    (5, 11),
    (6, 13),
    (7, 14),
    (8, 21),
    (9, 23),
    (10, 25),
    (11, 26),
    (12, 28),
    (13, 30),
    (14, 31),
    (15, 33),
];

/// K3 3.3V 驱动强度表（pinctrl-k1.c:365-382）。
pub const K3_DS_3V3: [(u32, u32); 16] = [
    (0, 3),
    (1, 5),
    (2, 7),
    (3, 9),
    (4, 11),
    (5, 13),
    (6, 15),
    (7, 17),
    (8, 25),
    (9, 27),
    (10, 29),
    (11, 31),
    (12, 33),
    (13, 35),
    (14, 37),
    (15, 38),
];

/// 目标 mA → 寄存器 val（pinctrl-k1.c:384-394 `spacemit_get_ds_value`）。
///
/// 线性扫描，返回**首个 mA ≥ 目标**的 val；若全部小于则 clamp 到最后一项。
pub fn ds_to_val(table: &[(u32, u32)], ma: u32) -> u32 {
    for &(val, threshold_ma) in table {
        if threshold_ma >= ma {
            return val;
        }
    }
    // 全部小于目标：clamp 到最后一项（pinctrl-k1.c:393）。
    table.last().map(|(val, _)| *val).unwrap_or(0)
}
