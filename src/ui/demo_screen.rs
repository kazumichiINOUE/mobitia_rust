use eframe::egui;

use crate::demo::DemoManager;

#[derive(Default)]
pub struct DemoScreen {}

impl DemoScreen {
    pub fn new() -> Self {
        Self {}
    }

    pub fn draw(&mut self, ui: &mut egui::Ui, demo_manager: &mut DemoManager) {
        demo_manager.update_and_draw(ui);
    }
}
