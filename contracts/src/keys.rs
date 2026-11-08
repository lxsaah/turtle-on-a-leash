//! Record keys. A misspelled key connects to nothing and fails silently, so
//! every participant takes them from here.

pub const MISSION: &str = "delivery.mission";
pub const WORLD_MAP: &str = "world.map";
pub const POLICY: &str = "motion.policy";
pub const POSE: &str = "turtle.pose";
pub const ROUTE_PROPOSAL: &str = "delivery.route_proposal";
pub const ROUTE_DECISION: &str = "delivery.route_decision";
pub const STATUS: &str = "delivery.status";
pub const CMD_VEL: &str = "turtle.cmd_vel";
pub const BUTTON: &str = "board.button.b1";
pub const BOARD_DIAG: &str = "mcu.diag";
pub const LINK_HEALTH: &str = "link.health";
