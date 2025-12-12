use eframe::egui;

pub mod rotating_scan;
pub mod expanding_ripple;
pub mod breathing_circle;
pub mod table;

use rotating_scan::RotatingScanDemo;
use expanding_ripple::ExpandingRippleDemo;
use breathing_circle::BreathingCircleDemo;
use table::TableDemo;

#[derive(PartialEq, Clone, Copy, Debug)]
pub enum DemoMode {
    RotatingScan,
    ExpandingRipple,
    BreathingCircle,
    Table,
}

pub struct DemoManager {
    pub(crate) mode: DemoMode,
    pub(crate) rotating_scan_demo: RotatingScanDemo,
    pub(crate) expanding_ripple_demo: ExpandingRippleDemo,
    pub(crate) breathing_circle_demo: BreathingCircleDemo,
    pub(crate) table_demo: TableDemo,
}

impl DemoManager {
    pub fn new() -> Self {
        Self {
            mode: DemoMode::RotatingScan,
            rotating_scan_demo: RotatingScanDemo::new(),
            expanding_ripple_demo: ExpandingRippleDemo::new(),
            breathing_circle_demo: BreathingCircleDemo::new(),
            table_demo: TableDemo::new(),
        }
    }

    pub fn set_mode(&mut self, mode: DemoMode) {
        self.mode = mode;
    }

    pub fn update_and_draw(&mut self, ui: &mut egui::Ui) {
        let heading_text = match self.mode {
            DemoMode::RotatingScan => "Rotating Scan Demo",
            DemoMode::ExpandingRipple => "Expanding Ripple Demo",
            DemoMode::BreathingCircle => "Breathing Circle Demo",
            DemoMode::Table => "Table Demo",
        };
        ui.heading(heading_text);

        ui.painter().rect_filled(
            ui.available_rect_before_wrap(),
            0.0,
            egui::Color32::from_rgb(20, 20, 20),
        );

        match self.mode {
            DemoMode::Table => self.table_demo.update_and_draw(ui),
            DemoMode::RotatingScan => self.rotating_scan_demo.update_and_draw(ui),
            DemoMode::ExpandingRipple => self.expanding_ripple_demo.update_and_draw(ui),
            DemoMode::BreathingCircle => self.breathing_circle_demo.update_and_draw(ui),
        }
    }
}
